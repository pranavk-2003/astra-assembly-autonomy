"""Typed recipe domain model. Building these from validated JSON is the only
place a raw dict is trusted; everything downstream works with these types."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from astra_core.geometry.pose import Pose


@dataclass(frozen=True)
class Shape:
    kind: str  # "box"
    size: np.ndarray  # (3,) full extents


@dataclass(frozen=True)
class Grasp:
    approach_vector: np.ndarray
    approach_distance: float
    grasp_offset_xyz: np.ndarray


@dataclass(frozen=True)
class Part:
    id: str
    shape: Shape
    source_pose: Pose
    assembly_pose: Pose
    grasp: Grasp


@dataclass(frozen=True)
class Joint:
    id: str
    type: str
    members: tuple[str, ...]
    pose: Pose
    approach_vector: np.ndarray
    approach_distance: float


@dataclass(frozen=True)
class Obstacle:
    id: str
    shape: Shape
    pose: Pose


@dataclass(frozen=True)
class Constraints:
    max_perception_translation_correction_m: float
    max_perception_rotation_correction_deg: float


@dataclass(frozen=True)
class Recipe:
    job_id: str
    frame_id: str
    units: str
    parts: tuple[Part, ...]
    joints: tuple[Joint, ...]
    obstacles: tuple[Obstacle, ...]
    constraints: Constraints

    def part(self, part_id: str) -> Part:
        for p in self.parts:
            if p.id == part_id:
                return p
        raise KeyError(f"unknown part_id {part_id!r} in job {self.job_id!r}")


@dataclass(frozen=True)
class PerceptionCorrection:
    job_id: str
    part_id: str
    confidence: float
    delta_translation_m: np.ndarray
    delta_rpy_deg: np.ndarray
    raw: dict = field(default_factory=dict, compare=False)
