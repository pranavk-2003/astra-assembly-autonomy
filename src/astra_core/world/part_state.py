"""Tracks nominal vs. observed/corrected pose per part, and attach state.
Keeping these distinct is what prevents a correction being applied twice
across re-plans (the assessment spec: "a correction is applied once, not compounded")."""
from __future__ import annotations

from dataclasses import dataclass

from astra_core.geometry.pose import Pose


@dataclass
class PartState:
    part_id: str
    nominal_pose: Pose
    observed_pose: Pose | None = None
    attached_to_gripper: bool = False

    @property
    def effective_pose(self) -> Pose:
        return self.observed_pose if self.observed_pose is not None else self.nominal_pose

    def apply_correction(self, corrected_pose: Pose) -> None:
        """Overwrites (does not compose onto) any prior observation - a fresh
        accepted correction always replaces, it never stacks."""
        self.observed_pose = corrected_pose
