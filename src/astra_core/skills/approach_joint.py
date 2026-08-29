"""ApproachJoint(): move to the joint's derived approach pose. Distinct name
from Approach() only to keep the trace log's skill field self-describing;
the underlying motion is identical."""
from __future__ import annotations

from astra_core.geometry.approach import derive_approach_pose
from astra_core.geometry.pose import Pose
from astra_core.skills.base import SkillContext, SkillOutcome, plan_and_execute


def approach_joint(ctx: SkillContext, joint_pose: Pose, approach_vector, approach_distance: float) -> SkillOutcome:
    standoff = derive_approach_pose(joint_pose, approach_vector, approach_distance)
    return plan_and_execute(ctx, "ApproachJoint", standoff)
