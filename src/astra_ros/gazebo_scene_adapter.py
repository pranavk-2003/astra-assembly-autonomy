"""ScenePort implementation that spawns/moves REAL, physically-simulated box
models in Gazebo (gz_sim) for every recipe part/obstacle - composed
alongside MoveItSceneAdapter (see CompositeSceneAdapter below) so MoveIt's
planning-scene collision model and Gazebo's own physics world both reflect
the same recipe geometry. Without this, parts/obstacles exist only as
abstract MoveIt/RViz collision geometry and are invisible in Gazebo itself
(confirmed live - the robot moved through empty space in the Gazebo view).

Shells out to ros_gz_sim's create/set_entity_pose CLI tools rather than
gz-transport Python bindings (not verified installed here), matching this
repo's existing pattern of using ROS2 CLI tools from Python where a stable
Python API isn't available. Confined to astra_ros/."""
from __future__ import annotations

import subprocess

from astra_core.geometry.pose import Pose
from astra_core.ports.scene_port import ScenePort
from astra_core.recipe.models import Shape


def _box_sdf(object_id: str, shape: Shape) -> str:
    sx, sy, sz = (float(v) for v in shape.size)
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{object_id}">
    <link name="body">
      <inertial>
        <mass>0.1</mass>
        <inertia><ixx>0.0001</ixx><ixy>0</ixy><ixz>0</ixz>
                  <iyy>0.0001</iyy><iyz>0</iyz><izz>0.0001</izz></inertia>
      </inertial>
      <visual name="visual">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material><ambient>0 0.8 0 1</ambient><diffuse>0 0.8 0 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>"""


class GazeboSceneAdapter(ScenePort):
    """Stage 1: spawn + pose-sync only. attach()/detach() do not yet create a
    real physical grip in Gazebo (that needs gz_sim's DetachableJoint system,
    dynamically loaded via gz-transport's /entity/system/add service - a
    separate, more involved piece not yet built; see
    docs/moveit2_integration_notes.md). detach() at least drops the object
    back at its place pose, matching MoveIt's own detach behavior."""

    def __init__(self) -> None:
        self._spawned: set[str] = set()

    def _spawn_or_move(self, object_id: str, shape: Shape, pose: Pose) -> None:
        rpy = pose.to_rpy()
        if object_id not in self._spawned:
            subprocess.run(
                [
                    "ros2", "run", "ros_gz_sim", "create",
                    "-string", _box_sdf(object_id, shape),
                    "-name", object_id,
                    "-x", str(pose.xyz[0]), "-y", str(pose.xyz[1]), "-z", str(pose.xyz[2]),
                    "-R", str(rpy[0]), "-P", str(rpy[1]), "-Y", str(rpy[2]),
                ],
                check=False,
                capture_output=True,
            )
            self._spawned.add(object_id)
        else:
            subprocess.run(
                [
                    "ros2", "run", "ros_gz_sim", "set_entity_pose",
                    "--name", object_id,
                    "--pos", str(pose.xyz[0]), str(pose.xyz[1]), str(pose.xyz[2]),
                    "--euler", str(rpy[0]), str(rpy[1]), str(rpy[2]),
                ],
                check=False,
                capture_output=True,
            )

    def add_object(self, object_id: str, shape: Shape, pose: Pose) -> None:
        self._spawn_or_move(object_id, shape, pose)

    def update_pose(self, object_id: str, pose: Pose) -> None:
        self._spawn_or_move(object_id, None, pose)  # shape unused once spawned

    def attach(self, object_id: str) -> None:
        pass  # Stage 2, not yet implemented

    def detach(self, object_id: str, pose: Pose) -> None:
        self.update_pose(object_id, pose)

    def allow_collision(self, object_id: str) -> None:
        pass  # MoveIt-side concept only; Gazebo has its own real contact physics

    def disallow_collision(self, object_id: str) -> None:
        pass


class CompositeSceneAdapter(ScenePort):
    """Fans every ScenePort call out to multiple adapters, so demo_gazebo.launch.py
    can drive BOTH MoveIt's planning scene and Gazebo's physical world from
    the same skills/pick.py, place.py calls - astra_core is unaware this is
    happening (it still just calls ctx.scene.X() once)."""

    def __init__(self, adapters: list[ScenePort]) -> None:
        self._adapters = adapters

    def add_object(self, object_id: str, shape: Shape, pose: Pose) -> None:
        for a in self._adapters:
            a.add_object(object_id, shape, pose)

    def update_pose(self, object_id: str, pose: Pose) -> None:
        for a in self._adapters:
            a.update_pose(object_id, pose)

    def attach(self, object_id: str) -> None:
        for a in self._adapters:
            a.attach(object_id)

    def detach(self, object_id: str, pose: Pose) -> None:
        for a in self._adapters:
            a.detach(object_id, pose)

    def allow_collision(self, object_id: str) -> None:
        for a in self._adapters:
            a.allow_collision(object_id)

    def disallow_collision(self, object_id: str) -> None:
        for a in self._adapters:
            a.disallow_collision(object_id)
