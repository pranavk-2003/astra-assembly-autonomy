"""Global scene transform (per the assessment spec's scoping note: "if the chosen arm cannot
reach the supplied poses, apply one documented global scene transform to the
whole cell"). Identity here - the Panda's ~0.85m reach comfortably covers the
recipe's x in [0.30, 0.64], y in [-0.35, 0.35] envelope with the base at the
recipe's world origin, so no transform is needed. Kept as an explicit,
single seam so a different arm/cell layout only changes this one function,
never individual targets."""
from __future__ import annotations

from astra_core.geometry.pose import Pose

# world_to_base: recipe "world" frame -> robot planning frame (panda_link0).
# Identity by measurement (see docstring); change ONLY here if a future robot
# choice needs an offset.
WORLD_TO_BASE_XYZ = (0.0, 0.0, 0.0)
WORLD_TO_BASE_RPY = (0.0, 0.0, 0.0)


def to_planning_frame(pose: Pose) -> Pose:
    if WORLD_TO_BASE_XYZ == (0.0, 0.0, 0.0) and WORLD_TO_BASE_RPY == (0.0, 0.0, 0.0):
        return pose
    offset = Pose.from_xyz_rpy(WORLD_TO_BASE_XYZ, WORLD_TO_BASE_RPY)
    return Pose(xyz=offset.xyz + pose.xyz, quat_xyzw=pose.quat_xyzw)
