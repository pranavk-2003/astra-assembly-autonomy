"""Waypoint derivation. Pre-grasp, pre-place, joint-approach and retreat poses
are all computed here from recipe parameters - none are stored (the assessment spec says
"Waypoints are derived, not stored")."""
from __future__ import annotations

import numpy as np

from astra_core.geometry.pose import Pose


def _normalize(vec) -> np.ndarray:
    v = np.asarray(vec, dtype=float)
    norm = np.linalg.norm(v)
    if norm < 1e-9:
        raise ValueError("approach_vector must be non-zero")
    return v / norm


def derive_grasp_pose(part_pose: Pose, grasp_offset_xyz) -> Pose:
    """Grasp point = part pose with grasp_offset_xyz applied in the part's own
    frame; gripper orientation matches the part's orientation at grasp."""
    grasp_xyz = part_pose.transform_point(grasp_offset_xyz)
    return Pose(xyz=grasp_xyz, quat_xyzw=part_pose.quat_xyzw)


def derive_approach_pose(target_pose: Pose, approach_vector, approach_distance: float) -> Pose:
    """target_pose (Ominus) approach_vector * approach_distance: stand off from
    the target along the negative approach direction, same orientation as the
    target. Used for pre-grasp, pre-place, joint-approach AND retreat (the
    retreat pose for a motion is the same standoff pose the motion approached
    from)."""
    v = _normalize(approach_vector)
    standoff_xyz = target_pose.xyz - v * approach_distance
    return Pose(xyz=standoff_xyz, quat_xyzw=target_pose.quat_xyzw)
