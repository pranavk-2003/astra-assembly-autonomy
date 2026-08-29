"""Pick(): descend to the derived grasp pose, attach the part (scene + world
+ execution all agree), then verify the grasp succeeded (R9's pick-
verification failure class)."""
from __future__ import annotations

from astra_core.geometry.approach import derive_grasp_pose
from astra_core.geometry.pose import Pose
from astra_core.skills.base import SkillContext, SkillOutcome, plan_and_execute


def pick(ctx: SkillContext, part_id: str, part_pose: Pose, grasp_offset_xyz) -> SkillOutcome:
    grasp_pose = derive_grasp_pose(part_pose, grasp_offset_xyz)
    # The final grasp descent necessarily puts the gripper around/overlapping
    # the part - a planner otherwise reports the goal as in collision with the
    # very object being grasped (R3/R5; see docs/moveit2_integration_notes.md).
    ctx.scene.allow_collision(part_id)
    outcome = plan_and_execute(ctx, "Pick", grasp_pose)
    if not outcome.success:
        return outcome

    ctx.scene.attach(part_id)
    ctx.execution.attach(part_id)
    ctx.world.attach(part_id)

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
