"""Global scene transform (per the assessment spec's scoping note: "if the chosen arm cannot
reach the supplied poses, apply one documented global scene transform to the
whole cell"). Identity here - the Panda's ~0.85m reach comfortably covers the
recipe's x in [0.30, 0.64], y in [-0.35, 0.35] envelope with the base at the
recipe's world origin, so no transform is needed. Kept as an explicit,
single seam so a different arm/cell layout only changes this one function,
never individual targets."""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

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


# Gripper-orientation calibration for MoveIt2 goal poses only (never for
# collision-object poses, which must keep the recipe's own part/obstacle
# orientation intact for correct collision modeling - see moveit_scene_adapter.py,
# which does NOT use this function).
#
# Root cause (see docs/moveit2_integration_notes.md): the recipe's part rpy is
# authored as the PART's own orientation, not a pre-computed gripper-tool-frame
# orientation. Per the assessment spec's pose-derivation formula, derived
# grasp/approach poses inherit that orientation unchanged (astra_core keeps
# this - it is correct per spec and is what its tests assert). Sending that
# orientation straight to the Panda produces a goal state that is IN COLLISION
# with the recipe's own obstacles/parts at typical approach standoff positions
# (confirmed live: identity orientation -> GOAL_STATE_INVALID once obstacles
# and parts are in the planning scene; a fixed "gripper pointing down"
# orientation plans successfully at the same xyz - see
# docs/moveit2_integration_notes.md for the live evidence).
#
# Fix, scoped to this adapter only (never astra_core): override the goal
# orientation with a down-facing gripper, ROLLED about the vertical (now-
# approach) axis by the part's own yaw, before handing a target to MoveIt2.
# This is a real robot/cell calibration, exactly analogous to WORLD_TO_BASE
# above, and belongs at the same seam.
#
# A first version used a FIXED down-facing quaternion (ignoring the part's
# yaw entirely). That planned and executed without collision, but visually
# drove the gripper straight through the part rather than straddling it -
# "collision-free" isn't "correctly aligned to grasp": every supplied recipe
# uses approach_vector [0, 0, -1] uniformly, so a fixed down-facing base
# orientation is the right choice for the approach AXIS, but the roll about
# that axis must still track the part's yaw so the gripper's finger-closing
# direction (perpendicular to the part's long axis) lines up with the part,
# and so the carried part's long axis matches its recipe orientation instead
# of a fixed one (this was also the root cause of the exclusion-zone collision
# on Retreat documented in docs/moveit2_integration_notes.md).
# Default kept for callers that do not pass a profile; the real value per
# robot lives in astra_ros.robot_profile.RobotProfile.grasp_base_rpy.
GRIPPER_DOWN_BASE = Rotation.from_euler("xyz", [np.pi, 0.0, 0.0])


def calibrate_gripper_orientation(pose: Pose, base_rpy=None) -> Pose:
    """Point the tool down at the work, rolled to follow the part's own yaw.

    `base_rpy` is the robot's own down-facing orientation - see
    RobotProfile.grasp_base_rpy. It differs between arms by the clocking of the
    gripper on the flange, which is what decides whether the jaws close across
    a part's width or along its length.
    """
    base = GRIPPER_DOWN_BASE if base_rpy is None else Rotation.from_euler("xyz", list(base_rpy))
    yaw = pose.to_rpy()[2]
    composed = Rotation.from_euler("z", yaw) * base
    return Pose(xyz=pose.xyz, quat_xyzw=composed.as_quat())
