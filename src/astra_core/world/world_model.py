"""World/Scene Manager: authoritative in-process record of every part and
obstacle collision geometry, independent of any planner. The planner adapter
mirrors this into MoveIt's PlanningScene; this module holds no ROS types."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from astra_core.geometry.pose import Pose
from astra_core.recipe.models import Obstacle, Recipe, Shape
from astra_core.world.part_state import PartState


@dataclass
class WorldModel:
    parts: dict[str, PartState] = field(default_factory=dict)
    obstacles: dict[str, Obstacle] = field(default_factory=dict)

    @staticmethod
    def from_recipe(recipe: Recipe) -> "WorldModel":
        world = WorldModel()
        for part in recipe.parts:
            world.parts[part.id] = PartState(part_id=part.id, nominal_pose=part.source_pose)
        for obstacle in recipe.obstacles:
            world.obstacles[obstacle.id] = obstacle
        return world

    def pose_of(self, part_id: str) -> Pose:
        return self.parts[part_id].effective_pose

    def attach(self, part_id: str) -> None:
        self.parts[part_id].attached_to_gripper = True

    def detach(self, part_id: str, place_pose: Pose) -> None:
        state = self.parts[part_id]
        state.attached_to_gripper = False
        state.observed_pose = place_pose

    def apply_perception_correction(self, part_id: str, corrected_pose: Pose) -> None:
        self.parts[part_id].apply_correction(corrected_pose)

    def clearance_height_for(self, pose: Pose, shape: Shape,
                             exempt: frozenset[str] = frozenset(),
                             margin_m: float = 0.02) -> float:
        """Height a part must be RAISED to so it clears every exclusion zone it
        currently overlaps, or 0.0 if it already clears them all.

        A part whose source pose lies inside an exclusion zone cannot have that
        zone enforced while it sits there - the constraint is violated before
        anything moves. Lifting it above the zone first makes the constraint
        satisfiable, so it can be enforced for the rest of the journey rather
        than waived for the whole job.

        Derived from the recipe's own geometry: obstacle tops and the part's
        own height. `exempt` names structure the robot is meant to work inside
        (a jig), which is not something to climb over.
        """
        lo, _ = _aabb(pose, shape)
        highest_top = 0.0
        for obstacle_id, obstacle in self.obstacles.items():
            if obstacle_id in exempt:
                continue
            if not _boxes_overlap(pose, shape, obstacle.pose, obstacle.shape):
                continue
            _, obstacle_hi = _aabb(obstacle.pose, obstacle.shape)
            highest_top = max(highest_top, float(obstacle_hi[2]))
        if highest_top == 0.0:
            return 0.0
        return max(0.0, highest_top + margin_m - float(lo[2]))

    def work_holding_obstacles(self, pose: Pose, shape: Shape) -> tuple[str, ...]:
        """Obstacle ids whose volume encloses a pose the robot is REQUIRED to
        reach with a part of this shape.

        Such an obstacle cannot be a no-go volume: it is work-holding
        structure - a jig or nest the part rests in or is assembled onto - and
        the manipulator must be able to reach into it to do the job at all.
        Treating it as a hard obstacle makes the recipe unplannable by
        construction (measured on the supplied recipes: a part's source pose
        lies inside one obstacle, its assembly pose inside another).

        Classified from geometry alone, so no obstacle id is ever named in
        source; obstacles that do not enclose a required pose keep full
        collision checking as genuine exclusion zones.
        """
        return tuple(
            obstacle_id
            for obstacle_id, obstacle in self.obstacles.items()
            if _boxes_overlap(pose, shape, obstacle.pose, obstacle.shape)
        )


def _aabb(pose: Pose, shape: Shape) -> tuple[np.ndarray, np.ndarray]:
    """Axis-aligned bounds of an oriented box, via its eight rotated corners."""
    half = np.asarray(shape.size, dtype=float) / 2.0
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    corners = (pose.rotation().as_matrix() @ (signs * half).T).T + np.asarray(pose.xyz)
    return corners.min(axis=0), corners.max(axis=0)


def _boxes_overlap(pose_a: Pose, shape_a: Shape, pose_b: Pose, shape_b: Shape) -> bool:
    a_lo, a_hi = _aabb(pose_a, shape_a)
    b_lo, b_hi = _aabb(pose_b, shape_b)
    return bool(np.all(np.minimum(a_hi, b_hi) - np.maximum(a_lo, b_lo) > 0.0))
