"""Applies an accepted correction. Always composes from the part's NOMINAL
pose (never the current effective pose) so repeated re-plans do not stack the
same correction (R8)."""
from __future__ import annotations

from astra_core.geometry.pose import Pose
from astra_core.geometry.transforms import compose_correction
from astra_core.recipe.models import PerceptionCorrection


def corrected_pose_from_nominal(nominal_pose: Pose, correction: PerceptionCorrection) -> Pose:
    return compose_correction(nominal_pose, correction.delta_translation_m, correction.delta_rpy_deg)
