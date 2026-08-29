"""ScenePort: collision-world mutation contract. The MoveIt2 adapter mirrors
these calls into a PlanningScene; astra_sim's mock just records them (enough
to assert R3's attach/detach correctness in tests without a simulator)."""
from __future__ import annotations

import abc

from astra_core.geometry.pose import Pose
from astra_core.recipe.models import Shape


class ScenePort(abc.ABC):
    @abc.abstractmethod
    def add_object(self, object_id: str, shape: Shape, pose: Pose) -> None: ...

    @abc.abstractmethod
    def update_pose(self, object_id: str, pose: Pose) -> None: ...

    @abc.abstractmethod
    def attach(self, object_id: str) -> None: ...

    @abc.abstractmethod
    def detach(self, object_id: str, pose: Pose) -> None: ...
