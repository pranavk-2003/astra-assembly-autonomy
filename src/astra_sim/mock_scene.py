"""Headless ScenePort implementation: records every call so a test can assert
a carried part was attached on pick and detached on place (R3)."""
from __future__ import annotations

from collections.abc import Sequence

from astra_core.geometry.pose import Pose
from astra_core.ports.scene_port import ScenePort
from astra_core.recipe.models import Shape


class MockScene(ScenePort):
    def __init__(self) -> None:
        self.objects: dict[str, tuple[Shape, Pose]] = {}
        self.attached_ids: set[str] = set()
        self.calls: list[tuple] = []

    def add_object(self, object_id: str, shape: Shape, pose: Pose,
                   movable: bool = True) -> None:
        self.objects[object_id] = (shape, pose)
        self.calls.append(("add", object_id))

    def update_pose(self, object_id: str, pose: Pose) -> None:
        shape, _ = self.objects[object_id]
        self.objects[object_id] = (shape, pose)
        self.calls.append(("update_pose", object_id))

    def attach(self, object_id: str) -> None:
        self.attached_ids.add(object_id)
        self.calls.append(("attach", object_id))

    def detach(self, object_id: str, pose: Pose) -> None:
        self.attached_ids.discard(object_id)
        self.update_pose(object_id, pose)
        self.calls.append(("detach", object_id))

    def allow_collision(self, object_id: str, with_ids: Sequence[str] = ()) -> None:
        self.calls.append(("allow_collision", object_id))

    def disallow_collision(self, object_id: str, with_ids: Sequence[str] = ()) -> None:
        self.calls.append(("disallow_collision", object_id))
