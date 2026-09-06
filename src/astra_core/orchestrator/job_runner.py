"""The Orchestrator: walks LOAD_JOB -> VALIDATE -> BUILD_WORLD, then for every
derived step, PLAN_SKILL -> EXECUTE_SKILL -> VERIFY -> NEXT_STEP, branching to
RECOVERY (retry/re-plan/re-observe/safe-pose/operator-pause) or ABORT on
failure, per the fixed policy in astra_core.recovery.policy. Nothing here
knows about a specific part, joint or obstacle id - it only knows Recipe,
Step and Pose."""
from __future__ import annotations

from dataclasses import dataclass

from astra_core.geometry.approach import derive_grasp_pose
from astra_core.orchestrator.job_context import JobContext
from astra_core.orchestrator.states import JobStatus, State
from astra_core.orchestrator.step_builder import Step, StepKind, build_steps
from astra_core.orchestrator.fsm import enter_state
from astra_core.perception.correction import corrected_pose_from_nominal
from astra_core.perception.gate import evaluate as evaluate_gate
from astra_core.recipe.models import PerceptionCorrection, Recipe
from astra_core.recovery.classifier import (
    classify_execution_failure,
    classify_perception_rejection,
    classify_pick_verify_failure,
    classify_plan_failure,
)
from astra_core.recovery.policy import RecoveryAction, is_terminal, next_action
from astra_core.skills.approach import approach
from astra_core.skills.approach_joint import approach_joint
from astra_core.skills.base import SkillContext, SkillOutcome
from astra_core.skills.pick import pick
from astra_core.skills.place import place
from astra_core.skills.retreat import retreat
from astra_core.world.world_model import WorldModel


# Held part must rise more than this during retreat to count as a real lift,
# not settling noise.
GRASP_LIFT_EPSILON_M = 0.002


@dataclass(frozen=True)
class JobResult:
    status: JobStatus
    steps_completed: int
    steps_total: int


def _dispatch(step: Step, recipe: Recipe, world: WorldModel, skill_ctx: SkillContext) -> SkillOutcome:
    if step.kind in (StepKind.PICK_APPROACH, StepKind.PICK, StepKind.PICK_RETREAT):
        part = recipe.part(step.part_id)
        part_pose = world.pose_of(step.part_id)
        grasp_pose = derive_grasp_pose(part_pose, part.grasp.grasp_offset_xyz)
        vec, dist = part.grasp.approach_vector, part.grasp.approach_distance
        if step.kind is StepKind.PICK_APPROACH:
            # Re-observe: part may have settled/shifted since its nominal pose.
            # No-op on backends that cannot sense.
            observed = skill_ctx.scene.observe_pose(step.part_id)
            if observed is not None:
                world.apply_perception_correction(step.part_id, observed)
                skill_ctx.scene.update_pose(step.part_id, observed)
                part_pose = world.pose_of(step.part_id)
                grasp_pose = derive_grasp_pose(part_pose, part.grasp.grasp_offset_xyz)
                skill_ctx.logger.log(
                    step="observe", part_id=step.part_id,
                    observed_xyz=[round(float(v), 4) for v in observed.xyz],
                )
            # Exempt gripper vs. part from approach onward, not just at grasp:
            # some recipes' approach distance is shorter than the gripper's
            # finger length, so a fingertip is already inside the part here.
            skill_ctx.scene.allow_collision(step.part_id)
            return approach(skill_ctx, grasp_pose, vec, dist)
        if step.kind is StepKind.PICK:
            return pick(skill_ctx, step.part_id, part_pose, part.grasp.grasp_offset_xyz)
        pose_before_lift = world.pose_of(step.part_id)
        # Retreat far enough to clear the exclusion zone the part sits in, not
        # merely the recipe's standoff distance (some recipes place a part
        # inside a zone taller than that distance). Height is derived from
        # recipe geometry, never hardcoded.
        clearance = world.clearance_height_for(
            pose_before_lift, part.shape, exempt=skill_ctx.work_holding
        )
        lift_dist = max(dist, clearance)
        if lift_dist > dist:
            skill_ctx.logger.log(
                step="transit_lift", part_id=step.part_id,
                recipe_distance_m=round(float(dist), 4),
                required_m=round(float(clearance), 4),
                reason="clear exclusion zone before transport",
            )
        outcome = retreat(skill_ctx, grasp_pose, vec, lift_dist)
        if outcome.success:
            # Physical grasp check: a held part has RISEN with the gripper.
            # The gripper only reports its close command finished, which is
            # also true when jaws shut on nothing.
            observed = skill_ctx.scene.observe_pose(step.part_id)
            if observed is not None:
                lift = float(observed.xyz[2]) - float(pose_before_lift.xyz[2])
                held = lift > GRASP_LIFT_EPSILON_M
                skill_ctx.logger.log(
                    step="verify_physical", skill="Retreat", part_id=step.part_id,
                    lift_m=round(lift, 4), held=bool(held),
                )
                if not held:
                    return SkillOutcome(
                        success=False,
                        plan_result=outcome.plan_result,
                        execution_result=outcome.execution_result,
                        pick_verified=False,
                    )
            # Restore collision checks now the part is lifted clear, so it
            # can't sail through the zone in transit. Work-holding structure
            # stays exempt: the part still has to be lowered into it.
            _enforce_clear_obstacles(step.part_id, part.shape, world, skill_ctx)
        return outcome

    if step.kind in (StepKind.PLACE_APPROACH, StepKind.PLACE, StepKind.PLACE_RETREAT):
        part = recipe.part(step.part_id)
        place_pose = part.assembly_pose
        vec, dist = part.grasp.approach_vector, part.grasp.approach_distance
        if step.kind is StepKind.PLACE_APPROACH:
            # Re-evaluate now the part has been carried clear of its source.
            _enforce_clear_obstacles(step.part_id, part.shape, world, skill_ctx)
            return approach(skill_ctx, place_pose, vec, dist)
        if step.kind is StepKind.PLACE:
            return place(skill_ctx, step.part_id, place_pose)
        # Gripper-vs-part exemption is never restored after place: the
        # recipe's retreat distance is shorter than the gripper's finger
        # length, so restoring it would strand the arm in collision at its
        # own start state. Arm links still collision-check against the part.
        return retreat(skill_ctx, place_pose, vec, dist)

    # JOINT_APPROACH
    joint = next(j for j in recipe.joints if j.id == step.joint_id)
    return approach_joint(skill_ctx, joint.pose, joint.approach_vector, joint.approach_distance)


def _enforce_clear_obstacles(part_id: str, shape, world: WorldModel,
                             skill_ctx: SkillContext) -> None:
    """Re-enable collision checking of a carried part against every obstacle
    it is currently clear of. A constraint the start state already violates
    can't be enforced, so this is re-run as the part moves."""
    overlapping = set(world.work_holding_obstacles(world.pose_of(part_id), shape))
    enforceable = [
        obstacle_id for obstacle_id in world.obstacles
        if obstacle_id not in skill_ctx.work_holding and obstacle_id not in overlapping
    ]
    if enforceable:
        skill_ctx.scene.disallow_collision(part_id, enforceable)
    skill_ctx.logger.log(
        step="transit_collision", part_id=part_id,
        enforced=sorted(enforceable), exempt_still_overlapping=sorted(overlapping),
    )


def _classify(outcome: SkillOutcome):
    if outcome.plan_result is not None and not outcome.plan_result.success:
        return classify_plan_failure(outcome.plan_result)
    if outcome.execution_result is not None and not outcome.execution_result.ok:
        return classify_execution_failure(outcome.execution_result)
    if outcome.pick_verified is False:
        return classify_pick_verify_failure()
    raise AssertionError("failed outcome carried no classifiable failure")


def apply_perception_correction(
    job_ctx: JobContext, correction: PerceptionCorrection, confidence_threshold: float = 0.90
) -> bool:
    """Runs the perception gate and, if accepted, updates the world model and
    the collision scene from the part's NOMINAL pose (R8). Returns whether the
    job should proceed (True) or has been paused/aborted (False)."""
    logger = job_ctx.skill_ctx.logger
    recipe = job_ctx.recipe
    gate_result = evaluate_gate(correction, recipe.constraints, confidence_threshold)
    logger.log(
        event="perception_gate",
        part_id=correction.part_id,
        confidence=correction.confidence,
        gate_accepted=gate_result.accepted,
        reasons=list(gate_result.reasons),
    )

    if gate_result.accepted:
        part = recipe.part(correction.part_id)
        corrected = corrected_pose_from_nominal(part.source_pose, correction)
        job_ctx.world.apply_perception_correction(correction.part_id, corrected)
        job_ctx.skill_ctx.scene.update_pose(correction.part_id, corrected)
        logger.log(
            event="perception_applied",
            part_id=correction.part_id,
            corrected_xyz=corrected.xyz.tolist(),
        )
        return True

    failure_class = classify_perception_rejection(gate_result)
    attempt = job_ctx.attempt_index(-1, failure_class.value)
    action = next_action(failure_class, attempt)
    job_ctx.record_failure(-1, failure_class.value)
    enter_state(logger, State.RECOVERY, failure_class=failure_class.value, action=action.value)

    if is_terminal(action):
        job_ctx.status = (
            JobStatus.ABORTED if action is RecoveryAction.ABORT else JobStatus.OPERATOR_PAUSED
        )
        enter_state(logger, State.ABORT if action is RecoveryAction.ABORT else State.RECOVERY,
                    final_status=job_ctx.status.value)
        return False

    # Non-terminal (RE_OBSERVE): this headless run has only one correction
    # message to evaluate, so it cannot actually re-observe. It escalates to
    # OPERATOR_PAUSED rather than silently proceeding on the unverified
    # nominal pose - documented limitation, see docs/recovery_policy.md.
    job_ctx.status = JobStatus.OPERATOR_PAUSED
    enter_state(logger, State.RECOVERY, final_status=job_ctx.status.value,
                note="re-observation not available in headless run; escalated")
    return False


def run_job(
    recipe: Recipe,
    skill_ctx: SkillContext,
    correction: PerceptionCorrection | None = None,
    confidence_threshold: float = 0.90,
) -> JobResult:
    logger = skill_ctx.logger
    enter_state(logger, State.LOAD_JOB, job_id=recipe.job_id)
    enter_state(logger, State.VALIDATE)  # schema validation already ran in load_recipe/load_correction

    world = WorldModel.from_recipe(recipe)
    skill_ctx.world = world  # skills reach the live world model only through skill_ctx
    for obstacle in recipe.obstacles:
        # True recipe geometry, unpadded - planning margin is applied inside
        # the planner adapter (see MoveItSceneAdapter), not on the simulated
        # body itself.
        skill_ctx.scene.add_object(
            obstacle.id, obstacle.shape, obstacle.pose,
            movable=False,   # fixed cell structure, never manipulated
        )
    for part in recipe.parts:
        skill_ctx.scene.add_object(part.id, part.shape, world.pose_of(part.id))

    # Classify obstacles by geometry: one enclosing a part's ASSEMBLY pose is
    # work-holding structure the arm must reach into, not a no-go volume.
    # Keyed on assembly pose only - keying on source pose too would exempt
    # every exclusion zone a part merely rests in, defeating obstacle
    # avoidance for the arm itself.
    work_holding: set[str] = set()
    for part in recipe.parts:
        work_holding.update(world.work_holding_obstacles(part.assembly_pose, part.shape))
    skill_ctx.work_holding = frozenset(work_holding)
    for obstacle_id in sorted(work_holding):
        skill_ctx.scene.allow_arm_collision(obstacle_id)
        logger.log(step="world", obstacle=obstacle_id, classified="work_holding")

    enter_state(logger, State.BUILD_WORLD)

    job_ctx = JobContext(recipe=recipe, world=world, skill_ctx=skill_ctx)

    if correction is not None:
        proceed = apply_perception_correction(job_ctx, correction, confidence_threshold)
        if not proceed:
            return JobResult(status=job_ctx.status, steps_completed=0, steps_total=0)

    steps = build_steps(recipe)
    step_index = 0
    while step_index < len(steps):
        step = steps[step_index]
        enter_state(logger, State.PLAN_SKILL, step_index=step_index, skill=step.label)
        outcome = _dispatch(step, recipe, world, skill_ctx)
        enter_state(logger, State.EXECUTE_SKILL, step_index=step_index, skill=step.label,
                    success=outcome.success)

        if outcome.success:
            enter_state(logger, State.VERIFY, step_index=step_index, skill=step.label, result="ok")
            enter_state(logger, State.NEXT_STEP, step_index=step_index)
            step_index += 1
            continue

        failure_class = _classify(outcome)
        attempt = job_ctx.attempt_index(step_index, failure_class.value)
        action = next_action(failure_class, attempt)
        job_ctx.record_failure(step_index, failure_class.value)
        enter_state(
            logger, State.RECOVERY, step_index=step_index, skill=step.label,
            failure_class=failure_class.value, action=action.value, attempt=attempt,
        )

        if is_terminal(action):
            job_ctx.status = (
                JobStatus.ABORTED if action is RecoveryAction.ABORT else JobStatus.OPERATOR_PAUSED
            )
            enter_state(
                logger, State.ABORT if action is RecoveryAction.ABORT else State.RECOVERY,
                step_index=step_index, final_status=job_ctx.status.value,
            )
            return JobResult(status=job_ctx.status, steps_completed=step_index, steps_total=len(steps))

        enter_state(logger, State.REPLAN_RETRY, step_index=step_index, action=action.value)
        # retry/re-plan/re-observe/safe-pose all mean "attempt this step again"

    job_ctx.status = JobStatus.COMPLETE
    enter_state(logger, State.COMPLETE, steps_completed=len(steps))
    return JobResult(status=JobStatus.COMPLETE, steps_completed=len(steps), steps_total=len(steps))
