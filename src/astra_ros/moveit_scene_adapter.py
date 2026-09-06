"""ScenePort implementation backed by MoveIt2's PlanningScene. This is one of
the two files in the whole repo allowed to import moveit types (per the assessment
spec, the Planner Adapter is the only place cuRobo/MoveIt2 types appear - the world
model side of that boundary lives here alongside moveit_planner_adapter.py)."""
from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from moveit.planning import MoveItPy
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive

from astra_core.geometry.pose import Pose
from astra_core.ports.scene_port import ScenePort
from astra_core.recipe.models import Shape
from astra_ros.robot_profile import PANDA, RobotProfile
from astra_ros.ros_conversions import to_pose_msg


def _pose_to_matrix(pose: Pose) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = pose.rotation().as_matrix()
    m[:3, 3] = pose.xyz
    return m


def _matrix_to_pose(m: np.ndarray) -> Pose:
    return Pose(xyz=m[:3, 3], quat_xyzw=Rotation.from_matrix(m[:3, :3]).as_quat())

# Which links may touch a grasped part, and which make up the whole chain,
# are properties of the robot - see astra_ros.robot_profile.RobotProfile.


def _box_collision_object(object_id: str, shape: Shape, pose: Pose, frame_id: str) -> CollisionObject:
    obj = CollisionObject()
    obj.id = object_id
    obj.header.frame_id = frame_id
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(v) for v in shape.size]
    obj.primitives = [primitive]
    obj.primitive_poses = [to_pose_msg(pose)]
    obj.operation = CollisionObject.ADD
    return obj


class MoveItSceneAdapter(ScenePort):
    def __init__(self, moveit_py: MoveItPy, robot: RobotProfile = PANDA,
                 frame_id: str | None = None,
                 obstacle_margin_m: float = 0.05) -> None:
        self._moveit_py = moveit_py
        self._robot = robot
        self._planning_group = robot.arm_hand_group
        # Recipe "world" maps onto the robot's own planning root.
        self._frame_id = frame_id or robot.base_frame
        self._obstacle_margin_m = obstacle_margin_m
        self._shapes: dict[str, Shape] = {}
        self._poses: dict[str, Pose] = {}
        # part id -> pose in the gripper frame, while carried
        self._attached_rel: dict[str, Pose] = {}
        self._arm_exempt_obstacles: set[str] = set()
        # part ids the GRIPPER is exempt against (ACM only, MoveIt still
        # checks arm links) - tracked separately so a non-link-aware planner
        # like cuRobo can mirror it. See CuRoboPlannerAdapter._obstacles().
        self._gripper_exempt_obstacles: set[str] = set()

    def add_object(self, object_id: str, shape: Shape, pose: Pose,
                   movable: bool = True) -> None:
        # Fixed structure is padded for planning only - covers both the
        # planner grazing an obstacle and trajectory-tracking lag. Workpieces
        # are never padded: the gripper needs their true surface to grasp.
        if not movable and self._obstacle_margin_m:
            shape = Shape(
                kind=shape.kind,
                size=np.asarray(shape.size, dtype=float) + 2.0 * self._obstacle_margin_m,
            )
        self._shapes[object_id] = shape
        self._poses[object_id] = pose
        with self._moveit_py.get_planning_scene_monitor().read_write() as scene:
            scene.apply_collision_object(_box_collision_object(object_id, shape, pose, self._frame_id))
            scene.current_state.update()

    def update_pose(self, object_id: str, pose: Pose) -> None:
        # Re-applying a CollisionObject with the same id and operation=ADD
        # moves it in place (MoveIt merges by id) - no separate "move" verb.
        self.add_object(object_id, self._shapes[object_id], pose)

    def attach(self, object_id: str) -> None:
        # Compute the part's pose RELATIVE to the gripper at the moment of
        # grasp (gripper_pose^-1 . part_world_pose) and attach it there, so
        # the carried part keeps its own orientation as the gripper moves -
        # omitting this defaults the body to identity relative to the attach
        # link (see docs/moveit2_integration_notes.md).
        with self._moveit_py.get_planning_scene_monitor().read_write() as scene:
            gripper_matrix = scene.current_state.get_global_link_transform(self._robot.attach_link)
            part_matrix = _pose_to_matrix(self._poses[object_id])
            relative_pose = _matrix_to_pose(np.linalg.inv(gripper_matrix) @ part_matrix)

            attached = AttachedCollisionObject()
            attached.link_name = self._robot.attach_link
            attached.object.header.frame_id = self._robot.attach_link
            attached.object.id = object_id
            attached.object.operation = CollisionObject.ADD
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [float(v) for v in self._shapes[object_id].size]
            attached.object.primitives = [primitive]
            attached.object.primitive_poses = [to_pose_msg(relative_pose)]
            self._attached_rel[object_id] = relative_pose
            attached.touch_links = list(self._robot.gripper_links)
            scene.process_attached_collision_object(attached)
            scene.current_state.update()

    def attached_world_pose(self, object_id: str) -> Pose | None:
        """Where a carried part is in the world right now, or None if it is
        not being carried. Recomputed from the live gripper transform and the
        pose captured at grasp, so it tracks the arm as it moves."""
        relative = self._attached_rel.get(object_id)
        if relative is None:
            return None
        with self._moveit_py.get_planning_scene_monitor().read_only() as scene:
            gripper = scene.current_state.get_global_link_transform(self._robot.attach_link)
        return _matrix_to_pose(gripper @ _pose_to_matrix(relative))

    def attached_ids(self) -> tuple[str, ...]:
        return tuple(self._attached_rel)

    def detach(self, object_id: str, pose: Pose) -> None:
        self._attached_rel.pop(object_id, None)
        with self._moveit_py.get_planning_scene_monitor().read_write() as scene:
            detached = AttachedCollisionObject()
            detached.link_name = self._robot.attach_link
            detached.object.id = object_id
            detached.object.operation = CollisionObject.REMOVE
            scene.process_attached_collision_object(detached)
            scene.current_state.update()
        # detaching drops it back into the world at the place pose
        self.update_pose(object_id, pose)

    def allow_collision(self, object_id: str, with_ids: Sequence[str] = ()) -> None:
        with self._moveit_py.get_planning_scene_monitor().read_write() as scene:
            acm = scene.allowed_collision_matrix
            for other in [*self._robot.gripper_links, *with_ids]:
                acm.set_entry(object_id, other, True)
            scene.current_state.update()
        self._gripper_exempt_obstacles.add(object_id)

    def allow_arm_collision(self, object_id: str) -> None:
        with self._moveit_py.get_planning_scene_monitor().read_write() as scene:
            acm = scene.allowed_collision_matrix
            for link in self._robot.arm_links:
                acm.set_entry(object_id, link, True)
            scene.current_state.update()
        # Tracked separately so a non-MoveIt planner (cuRobo) can mirror the
        # same exemption - it plans against its own obstacle list, built from
        # this scene's boxes, and has no way to read MoveIt's ACM itself. See
        # CuRoboPlannerAdapter._obstacles().
        self._arm_exempt_obstacles.add(object_id)

    def disallow_arm_collision(self, object_id: str) -> None:
        with self._moveit_py.get_planning_scene_monitor().read_write() as scene:
            acm = scene.allowed_collision_matrix
            for link in self._robot.arm_links:
                acm.set_entry(object_id, link, False)
            scene.current_state.update()
        self._arm_exempt_obstacles.discard(object_id)

    def disallow_collision(self, object_id: str, with_ids: Sequence[str] = ()) -> None:
        # Only re-enforces the given obstacles (what job_runner calls this
        # for, mid-transit, as a part clears each one) - never the gripper
        # itself. The gripper exemption from allow_collision is permanent for
        # the rest of the job once a part has been grasped (see job_runner's
        # PLACE_RETREAT handling); un-exempting it here broke retreat right
        # after place, since the part becomes a world obstacle again but the
        # gripper is still resting on it at the retreat's start state.
        with self._moveit_py.get_planning_scene_monitor().read_write() as scene:
            acm = scene.allowed_collision_matrix
            for other in with_ids:
                acm.set_entry(object_id, other, False)
            scene.current_state.update()
