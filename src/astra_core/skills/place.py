"""Place(): descend to the target place pose, then detach - the carried part
stops colliding with the gripper and starts existing at the place pose in the
world model and collision scene simultaneously (R3)."""
from __future__ import annotations

from astra_core.geometry.pose import Pose
from astra_core.skills.base import SkillContext, SkillOutcome, plan_and_execute


def place(ctx: SkillContext, part_id: str, place_pose: Pose) -> SkillOutcome:
    outcome = plan_and_execute(ctx, "Place", place_pose)
    if not outcome.success:
        return outcome

    ctx.scene.detach(part_id, place_pose)
    ctx.execution.detach(part_id)
    ctx.world.detach(part_id, place_pose)
    return outcome
