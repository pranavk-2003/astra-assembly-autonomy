"""Pose representation. Rotation is stored as a quaternion internally so that
repeated composition (nominal -> correction -> re-plan) never accumulates the
gimbal/wrap-around error that plain Euler-angle addition would."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class Pose:
    xyz: np.ndarray  # shape (3,)
    quat_xyzw: np.ndarray  # shape (4,)

    def __post_init__(self) -> None:
        object.__setattr__(self, "xyz", np.asarray(self.xyz, dtype=float))
        object.__setattr__(self, "quat_xyzw", np.asarray(self.quat_xyzw, dtype=float))

    @staticmethod
    def from_xyz_rpy(xyz, rpy) -> "Pose":
        """rpy in radians, extrinsic XYZ (matches the recipe convention)."""
        quat = Rotation.from_euler("xyz", rpy, degrees=False).as_quat()
        return Pose(xyz=np.asarray(xyz, dtype=float), quat_xyzw=quat)

    @staticmethod
    def identity() -> "Pose":
        return Pose(xyz=np.zeros(3), quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]))

    def to_rpy(self) -> np.ndarray:
        return Rotation.from_quat(self.quat_xyzw).as_euler("xyz", degrees=False)

    def rotation(self) -> Rotation:
        return Rotation.from_quat(self.quat_xyzw)

    def translated(self, delta_xyz) -> "Pose":
        return Pose(xyz=self.xyz + np.asarray(delta_xyz, dtype=float), quat_xyzw=self.quat_xyzw)

    def rotated_world(self, delta_rotation: Rotation) -> "Pose":
        """Apply an additional rotation expressed in the world/target frame."""
        new_rot = delta_rotation * self.rotation()
        return Pose(xyz=self.xyz, quat_xyzw=new_rot.as_quat())

    def transform_point(self, point_local) -> np.ndarray:
        """Map a point given in this pose's local frame into the parent frame."""
        return self.xyz + self.rotation().apply(np.asarray(point_local, dtype=float))

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Pose):
            return NotImplemented
        return bool(
            np.allclose(self.xyz, other.xyz, atol=1e-9)
            and (
                np.allclose(self.quat_xyzw, other.quat_xyzw, atol=1e-9)
                or np.allclose(self.quat_xyzw, -other.quat_xyzw, atol=1e-9)
            )
        )
