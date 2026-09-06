"""Pick(): descend to the derived grasp pose, attach the part (scene + world
+ execution all agree), then verify the grasp succeeded (R9's pick-
verification failure class)."""
from __future__ import annotations

from astra_core.geometry.approach import derive_grasp_pose
from astra_core.geometry.pose import Pose
from astra_core.skills.base import SkillContext, SkillOutcome, obstacle_ids, plan_and_execute


def pick(ctx: SkillContext, part_id: str, part_pose: Pose, grasp_offset_xyz) -> SkillOutcome:
    grasp_pose = derive_grasp_pose(part_pose, grasp_offset_xyz)
    # The final grasp descent necessarily puts the gripper around/overlapping
    # the part - a planner otherwise reports the goal as in collision with the
    # very object being grasped (R3/R5; see docs/moveit2_integration_notes.md).
    # Open before descending: the fingers must be wider than the part before
    # they close around it, and nothing else in the sequence opens them - so
    # without this the grasp starts from wherever the previous step left the
    # gripper (measured on real physics: part-way closed at spawn, never
    # reopened). No-op on backends without an actuated gripper.
    ctx.execution.open_gripper()
    ctx.scene.allow_collision(part_id, obstacle_ids(ctx))
    outcome = plan_and_execute(ctx, "Pick", grasp_pose)
    if not outcome.success:
        return outcome

    ctx.scene.attach(part_id)
    # Re-assert the exemption AFTER attaching: attaching rebuilds the part as
    # a body carried by the gripper rather than a world object, which drops
    # the entries set above - so the carried part would start colliding with
    # the very obstacles the recipe deliberately routes it through, exactly
    # when it begins moving. The arm's own checks against them are untouched.
    ctx.scene.allow_collision(part_id, obstacle_ids(ctx))
    ctx.execution.attach(part_id)
    ctx.world.attach(part_id)

    # NOTE: whether the part is REALLY held cannot be judged here - nothing has
    # lifted yet. The gripper only reports that its close command completed,
    # which it does whether or not anything is between the jaws. The physical
    # check happens after the retreat, once there has been a lift to measure
    # (see the PICK_RETREAT branch in the orchestrator).
    verified = ctx.execution.verify_grasp(part_id)
    ctx.logger.log(step="verify", skill="Pick", part_id=part_id, pick_verified=verified)
    if not verified:
        return SkillOutcome(
            success=False,
            plan_result=outcome.plan_result,
            execution_result=outcome.execution_result,
            pick_verified=False,
        )
    return SkillOutcome(
        success=True,
        plan_result=outcome.plan_result,
        execution_result=outcome.execution_result,
        pick_verified=True,
    )
