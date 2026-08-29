"""Approach(): move to the standoff pose derived from a target + approach
vector/distance. Used ahead of Pick, Place and ApproachJoint alike."""
from __future__ import annotations

from astra_core.geometry.approach import derive_approach_pose
from astra_core.geometry.pose import Pose
from astra_core.skills.base import SkillContext, SkillOutcome, plan_and_execute


def approach(ctx: SkillContext, target: Pose, approach_vector, approach_distance: float) -> SkillOutcome:
    standoff = derive_approach_pose(target, approach_vector, approach_distance)
    return plan_and_execute(ctx, "Approach", standoff)
