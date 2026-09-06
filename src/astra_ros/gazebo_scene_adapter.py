"""ScenePort implementation that spawns/moves real, physically-simulated box
models in Gazebo (gz_sim) for every recipe part/obstacle - composed alongside
MoveItSceneAdapter (see CompositeSceneAdapter below) so MoveIt's planning
scene and Gazebo's physics world both reflect the same recipe geometry.

Shells out to ros_gz_sim's CLI tools rather than gz-transport Python
bindings (not verified installed here). Confined to astra_ros/."""
from __future__ import annotations

from collections.abc import Sequence

import subprocess
import sys
import time

from astra_core.geometry.pose import Pose
from astra_core.ports.scene_port import ScenePort
from astra_core.recipe.models import Shape


# Underside of every part in the supplied recipes - the height a work surface
# must present for parts to rest exactly at their recipe poses. Derived from
# source_pose.z - size.z/2, which is 0.0825 for both parts in both variants.
WORKTABLE_TOP_Z = 0.0825
WORKTABLE_SIZE = (1.6, 1.6, 0.05)


def _box_sdf(object_id: str, shape: Shape, *, static: bool, mass: float = 0.1) -> str:
    """A real rigid body for Gazebo. Parts are dynamic so the gripper can
    actually grasp them - static bodies are effectively infinite mass and
    stall the fingers past their joint limits. Friction is set explicitly
    since gz's default is too low to hold a grasped part in the jaws."""
    sx, sy, sz = (float(v) for v in shape.size)
    # Thin-box inertia tensor.
    ixx = mass * (sy * sy + sz * sz) / 12.0
    iyy = mass * (sx * sx + sz * sz) / 12.0
    izz = mass * (sx * sx + sy * sy) / 12.0
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{object_id}">
    <static>{'true' if static else 'false'}</static>
    <link name="body">
      <inertial>
        <mass>{mass}</mass>
        <inertia><ixx>{ixx}</ixx><ixy>0</ixy><ixz>0</ixz>
                 <iyy>{iyy}</iyy><iyz>0</iyz><izz>{izz}</izz></inertia>
      </inertial>
      <visual name="visual">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material><ambient>0 0.8 0 1</ambient><diffuse>0 0.8 0 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <surface>
          <!-- Matched to the gripper pads (grip_surface macro in
               config/ur_gazebo.urdf.xacro) - both surfaces need friction or
               the part slides straight out of the jaws. -->
          <friction>
            <ode><mu>2.0</mu><mu2>2.0</mu2></ode>
          </friction>
          <contact>
            <ode><kp>1e6</kp><kd>50</kd><min_depth>0.0005</min_depth><max_vel>0.0</max_vel></ode>
          </contact>
        </surface>
      </collision>
    </link>
  </model>
</sdf>"""


def _worktable_sdf() -> str:
    """The cell's work surface. Top is placed at the parts' own underside
    height, so they rest exactly where the recipe puts them."""
    sx, sy, sz = WORKTABLE_SIZE
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="worktable">
    <static>true</static>
    <link name="body">
      <visual name="visual">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material><ambient>0.4 0.35 0.3 1</ambient><diffuse>0.5 0.45 0.4 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <surface>
          <friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction>
        </surface>
      </collision>
    </link>
  </model>
</sdf>"""


class GazeboSceneAdapter(ScenePort):
    """Spawn + pose-sync. attach()/detach() are no-ops - grasping is real
    contact/friction physics, not a scripted joint (see attach() below)."""

    def __init__(self, world: str = "empty") -> None:
        self._spawned: set[str] = set()
        self._worktable_spawned = False
        self._world = world
        self._pending_spawns: list[subprocess.Popen] = []

    # A hung CLI call must never block the autonomy layer, so every call is
    # bounded and a miss is reported rather than awaited.
    _CLI_TIMEOUT_S = 10.0

    # How long the freshly spawned bodies get to come to rest before the job
    # reads a pose back or commands a move.
    _SETTLE_S = 1.5

    def _run(self, argv: list[str], what: str) -> None:
        try:
            result = subprocess.run(
                argv, check=False, capture_output=True, timeout=self._CLI_TIMEOUT_S
            )
        except subprocess.TimeoutExpired:
            print(f"[gazebo_scene] {what} timed out after {self._CLI_TIMEOUT_S}s; "
                  "Gazebo view may be stale", file=sys.stderr)
            return
        if result.returncode != 0:
            print(f"[gazebo_scene] {what} failed (rc={result.returncode}); "
                  "Gazebo view may be stale", file=sys.stderr)

    def _spawn_async(self, argv: list[str]) -> None:
        """Fire a spawn without waiting - parts and obstacles are independent
        of each other, so they all go out at once rather than one at a time.
        Drained by _settle() before anything reads the scene back."""
        self._pending_spawns.append(subprocess.Popen(argv))

    def _settle(self) -> None:
        """Wait for every in-flight spawn to land, then for the bodies to come
        to rest. Parts are dynamic: they are created at their recipe pose and
        drop the last fraction of a millimetre onto the table, so reading a
        pose back (or commanding a move) before this is reading a body that is
        still falling."""
        if not self._pending_spawns:
            return
        for process in self._pending_spawns:
            try:
                process.wait(timeout=self._CLI_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                process.kill()
                print("[gazebo_scene] a spawn did not complete in "
                      f"{self._CLI_TIMEOUT_S}s; Gazebo view may be incomplete",
                      file=sys.stderr)
        self._pending_spawns.clear()
        time.sleep(self._SETTLE_S)

    def _ensure_worktable(self) -> None:
        """Spawned synchronously and before any part. The parts' underside sits
        exactly at the table top, so a part created while the table does not
        yet exist free-falls instead of resting at its recipe pose - which then
        moves the grasp target out from under the gripper."""
        if self._worktable_spawned:
            return
        self._worktable_spawned = True
        z = WORKTABLE_TOP_Z - WORKTABLE_SIZE[2] / 2.0
        self._run(
            ["ros2", "run", "ros_gz_sim", "create",
             "-string", _worktable_sdf(), "-name", "worktable",
             "-x", "0", "-y", "0", "-z", str(z)],
            "spawn worktable",
        )

    def _spawn_or_move(self, object_id: str, shape: Shape, pose: Pose,
                       movable: bool = True) -> None:
        rpy = pose.to_rpy()
        if object_id not in self._spawned:
            self._ensure_worktable()
            self._spawn_async(
                [
                    "ros2", "run", "ros_gz_sim", "create",
                    "-string", _box_sdf(object_id, shape, static=not movable),
                    "-name", object_id,
                    "-x", str(pose.xyz[0]), "-y", str(pose.xyz[1]), "-z", str(pose.xyz[2]),
                    "-R", str(rpy[0]), "-P", str(rpy[1]), "-Y", str(rpy[2]),
                ],
            )
            self._spawned.add(object_id)
        else:
            self._set_pose(object_id, pose)

    def _set_pose(self, object_id: str, pose: Pose) -> None:
        """Move an already-spawned model via gz-transport's own set_pose.
        Not `ros2 run ros_gz_sim set_entity_pose`: that wrapper never returns
        for the static models used here."""
        self._settle()  # cannot move a model whose spawn is still in flight
        qx, qy, qz, qw = (float(v) for v in pose.quat_xyzw)
        x, y, z = (float(v) for v in pose.xyz)
        request = (
            f'name: "{object_id}", '
            f"position: {{x: {x}, y: {y}, z: {z}}}, "
            f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}"
        )
        self._run(
            [
                "gz", "service", "-s", f"/world/{self._world}/set_pose",
                "--reqtype", "gz.msgs.Pose",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "3000",
                "--req", request,
            ],
            f"move {object_id}",
        )

    def observe_pose(self, object_id: str) -> Pose | None:
        """The object's actual pose in the simulator, or None if unreadable.
        Stands in for a perception sensor: parts are dynamic bodies, so where
        one really is and where the recipe says it is can differ once it has
        settled or been nudged."""
        self._settle()  # never read a body that is still spawning or falling
        try:
            result = subprocess.run(
                ["gz", "model", "-m", object_id, "-p"],
                check=False, capture_output=True, text=True,
                timeout=self._CLI_TIMEOUT_S,
            )
        except subprocess.TimeoutExpired:
            return None
        if result.returncode != 0:
            return None
        # "Pose [ XYZ (m) ] [ RPY (rad) ]:" then a line of xyz, then rpy.
        lines = [ln.strip() for ln in result.stdout.splitlines()]
        for i, line in enumerate(lines):
            if line.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
                try:
                    xyz = [float(v) for v in lines[i].strip("[]").split()]
                    rpy = [float(v) for v in lines[i + 1].strip("[]").split()]
                except ValueError:
                    continue
                if len(xyz) == 3 and len(rpy) == 3:
                    return Pose.from_xyz_rpy(xyz, rpy)
        return None

    def add_object(self, object_id: str, shape: Shape, pose: Pose,
                   movable: bool = True) -> None:
        self._spawn_or_move(object_id, shape, pose, movable)

    def update_pose(self, object_id: str, pose: Pose) -> None:
        self._spawn_or_move(object_id, None, pose)  # shape unused once spawned

    def attach(self, object_id: str) -> None:
        # Nothing to do: the gripper is physically closed on the part, so
        # contact and friction carry it.
        pass

    def detach(self, object_id: str, pose: Pose) -> None:
        # Nothing to do: physics decides where the part ends up, which is
        # what shows whether the grasp actually worked.
        pass

    def allow_collision(self, object_id: str, with_ids: Sequence[str] = ()) -> None:
        pass  # MoveIt-side concept only; Gazebo has its own real contact physics

    def disallow_collision(self, object_id: str, with_ids: Sequence[str] = ()) -> None:
        pass


class CompositeSceneAdapter(ScenePort):
    """Fans every ScenePort call out to multiple adapters, so demo_gazebo.launch.py
    can drive BOTH MoveIt's planning scene and Gazebo's physical world from
    the same skills/pick.py, place.py calls - astra_core is unaware this is
    happening (it still just calls ctx.scene.X() once)."""

    def __init__(self, adapters: list[ScenePort]) -> None:
        self._adapters = adapters

    def add_object(self, object_id: str, shape: Shape, pose: Pose,
                   movable: bool = True) -> None:
        for a in self._adapters:
            a.add_object(object_id, shape, pose, movable)

    def update_pose(self, object_id: str, pose: Pose) -> None:
        for a in self._adapters:
            a.update_pose(object_id, pose)

    def attach(self, object_id: str) -> None:
        for a in self._adapters:
            a.attach(object_id)

    def detach(self, object_id: str, pose: Pose) -> None:
        for a in self._adapters:
            a.detach(object_id, pose)

    def allow_collision(self, object_id: str, with_ids: Sequence[str] = ()) -> None:
        for a in self._adapters:
            a.allow_collision(object_id, with_ids)

    def disallow_collision(self, object_id: str, with_ids: Sequence[str] = ()) -> None:
        for a in self._adapters:
            a.disallow_collision(object_id, with_ids)

    def observe_pose(self, object_id: str) -> Pose | None:
        for adapter in self._adapters:
            observe = getattr(adapter, "observe_pose", None)
            if observe is None:
                continue
            pose = observe(object_id)
            if pose is not None:
                return pose
        return None

    def sync_view(self) -> None:
        # Deliberately empty: a carried part's pose is the physics engine's
        # to decide, not MoveIt's.
        pass

    def allow_arm_collision(self, object_id: str) -> None:
        for a in self._adapters:
            a.allow_arm_collision(object_id)

    def disallow_arm_collision(self, object_id: str) -> None:
        for a in self._adapters:
            a.disallow_arm_collision(object_id)
