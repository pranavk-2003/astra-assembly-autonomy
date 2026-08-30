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
    # Collision checking against the part is deliberately NOT restored here.
    # The gripper is still closed around it at the instant of release, so
    # re-enabling the check would make the very next motion - withdrawing
    # from the part - unplannable from a start state that is in collision by
    # definition (observed live: contacts against the hand and wrist). The
    # orchestrator restores it once the retreat has actually withdrawn.
    # Part-vs-obstacle stays exempt permanently: the part now RESTS in the
    # work-holding structure it was assembled onto and overlaps it for good.
    return outcome
