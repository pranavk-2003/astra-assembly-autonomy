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
            # Exempt the gripper against the part it is deliberately moving to
            # envelop, from the approach onward rather than only at the grasp.
            # A recipe may specify an approach distance shorter than the
            # gripper's own finger length (one supplied part does: 0.1 m of
            # standoff against ~0.103 m fingers), which puts a fingertip
            # inside the part at the approach pose itself - verified live as
            # a finger/part contact that blocked the approach before pick()
            # had granted any exemption.
            skill_ctx.scene.allow_collision(step.part_id)
            return approach(skill_ctx, grasp_pose, vec, dist)
        if step.kind is StepKind.PICK:
            return pick(skill_ctx, step.part_id, part_pose, part.grasp.grasp_offset_xyz)
        return retreat(skill_ctx, grasp_pose, vec, dist)

    if step.kind in (StepKind.PLACE_APPROACH, StepKind.PLACE, StepKind.PLACE_RETREAT):
        part = recipe.part(step.part_id)
        place_pose = part.assembly_pose
        vec, dist = part.grasp.approach_vector, part.grasp.approach_distance
        if step.kind is StepKind.PLACE_APPROACH:
            return approach(skill_ctx, place_pose, vec, dist)
        if step.kind is StepKind.PLACE:
            return place(skill_ctx, step.part_id, place_pose)
        # NOTE: gripper-vs-part checking is deliberately never restored for a
        # placed part. Withdrawing by the recipe's own retreat distance leaves
        # the fingertips ~1 mm short of clearing the part's top face (that
        # distance is shorter than the gripper's finger length), so restoring
        # the check strands the arm in a start state that is in collision and
        # nothing further can plan. Only the GRIPPER links stay exempt - every
        # arm link still collision-checks against the placed part, so the arm
        # cannot sweep through it, and the part is in its final assembled
        # position by this point.
        return retreat(skill_ctx, place_pose, vec, dist)

    # JOINT_APPROACH
    joint = next(j for j in recipe.joints if j.id == step.joint_id)
    return approach_joint(skill_ctx, joint.pose, joint.approach_vector, joint.approach_distance)


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
        skill_ctx.scene.add_object(obstacle.id, obstacle.shape, obstacle.pose)
    for part in recipe.parts:
        skill_ctx.scene.add_object(part.id, part.shape, world.pose_of(part.id))

    # Classify obstacles by geometry: one that encloses a pose the robot is
    # required to reach is work-holding structure, not an exclusion zone, and
    # the arm must be able to enter it or the recipe cannot be executed at all
    # (verified against the planner: reaching an assembly pose inside such an
    # obstacle puts a forearm link through it). Obstacles enclosing no required
    # pose are untouched and keep full collision checking.
    work_holding: set[str] = set()
    for part in recipe.parts:
        for pose in (part.source_pose, part.assembly_pose):
            work_holding.update(world.work_holding_obstacles(pose, part.shape))
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
        # loop again on the SAME step_index - retry/re-plan/re-observe/safe-pose
        # all mean "attempt this step again"; the fault queue determines when
        # (or whether) the mock stops failing.

    job_ctx.status = JobStatus.COMPLETE
    enter_state(logger, State.COMPLETE, steps_completed=len(steps))
    return JobResult(status=JobStatus.COMPLETE, steps_completed=len(steps), steps_total=len(steps))
