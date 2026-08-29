"""Correction composition: a perception delta is always composed from the
NOMINAL pose, never from the current effective pose, so re-planning after an
accepted correction never double-applies it (R8 / the nominal-vs-observed
constraint)."""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from astra_core.geometry.pose import Pose


def compose_correction(nominal: Pose, delta_translation_m, delta_rpy_deg) -> Pose:
    """delta_translation_m: xyz in metres, world frame.
    delta_rpy_deg: intrinsic-XYZ rotation IN DEGREES (correction-message unit,
    distinct from the recipe's radians rpy) applied in the world frame."""
    delta_rot = Rotation.from_euler("xyz", np.asarray(delta_rpy_deg, dtype=float), degrees=True)
    corrected = nominal.translated(delta_translation_m)
    return corrected.rotated_world(delta_rot)


def translation_delta_m(a: Pose, b: Pose) -> np.ndarray:
    return b.xyz - a.xyz


def rotation_delta_deg(a: Pose, b: Pose) -> np.ndarray:
    """Smallest-angle rotation taking a's orientation to b's, as intrinsic XYZ
    Euler degrees (for gate comparisons against a per-axis or magnitude bound)."""
    delta = b.rotation() * a.rotation().inv()
    return delta.as_euler("xyz", degrees=True)


def rotation_delta_magnitude_deg(a: Pose, b: Pose) -> float:
    """Single-number rotation magnitude (rotation-vector angle), robust to
    axis choice - used for the perception gate's rotation bound."""
    delta = b.rotation() * a.rotation().inv()
    return float(np.degrees(np.linalg.norm(delta.as_rotvec())))
