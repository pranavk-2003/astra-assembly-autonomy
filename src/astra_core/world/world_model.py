"""World/Scene Manager: authoritative in-process record of every part and
obstacle collision geometry, independent of any planner. The planner adapter
mirrors this into MoveIt's PlanningScene; this module holds no ROS types."""
from __future__ import annotations

from dataclasses import dataclass, field

from astra_core.geometry.pose import Pose
from astra_core.recipe.models import Obstacle, Recipe
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
