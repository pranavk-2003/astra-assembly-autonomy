"""Retreat(): move back to the standoff pose for the position just occupied -
symmetric with Approach, same derivation, no separate "retreat" geometry."""
from __future__ import annotations

from astra_core.geometry.approach import derive_approach_pose
from astra_core.geometry.pose import Pose
from astra_core.skills.base import SkillContext, SkillOutcome, plan_and_execute


def retreat(ctx: SkillContext, current: Pose, approach_vector, approach_distance: float) -> SkillOutcome:
    standoff = derive_approach_pose(current, approach_vector, approach_distance)
    return plan_and_execute(ctx, "Retreat", standoff)
