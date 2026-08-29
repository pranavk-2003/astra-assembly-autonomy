"""Perception acceptance gate (the assessment spec's perception-gate rule): confidence AND
both magnitude bounds must pass, or the correction is rejected wholesale -
never partially applied."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from astra_core.recipe.models import Constraints, PerceptionCorrection

DEFAULT_CONFIDENCE_THRESHOLD = 0.90


@dataclass(frozen=True)
class GateResult:
    accepted: bool
    reasons: tuple[str, ...]  # empty when accepted


def evaluate(
    correction: PerceptionCorrection,
    constraints: Constraints,
    confidence_threshold: float = DEFAULT_CONFIDENCE_THRESHOLD,
) -> GateResult:
    reasons: list[str] = []

    if correction.confidence < confidence_threshold:
        reasons.append(
            f"low_confidence: {correction.confidence:.3f} < {confidence_threshold:.3f}"
        )

    translation_mag = float(np.linalg.norm(correction.delta_translation_m))
    if translation_mag > constraints.max_perception_translation_correction_m:
        reasons.append(
            "translation_out_of_bounds: "
            f"{translation_mag:.4f} m > {constraints.max_perception_translation_correction_m:.4f} m"
        )

    rotation_mag = float(np.max(np.abs(correction.delta_rpy_deg)))
    if rotation_mag > constraints.max_perception_rotation_correction_deg:
        reasons.append(
            "rotation_out_of_bounds: "
            f"{rotation_mag:.2f} deg > {constraints.max_perception_rotation_correction_deg:.2f} deg"
        )

    return GateResult(accepted=not reasons, reasons=tuple(reasons))
