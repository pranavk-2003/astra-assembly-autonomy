"""ScenePort implementation backed by MoveIt2's PlanningScene. This is one of
the two files in the whole repo allowed to import moveit types (per the assessment
spec, the Planner Adapter is the only place cuRobo/MoveIt2 types appear - the world
model side of that boundary lives here alongside moveit_planner_adapter.py)."""
from __future__ import annotations

from moveit.planning import MoveItPy
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive

from astra_core.geometry.pose import Pose
from astra_core.ports.scene_port import ScenePort
from astra_core.recipe.models import Shape
from astra_ros.ros_conversions import PLANNING_FRAME, to_pose_msg

# Gripper links that must be allowed to touch/overlap a part during the final
# grasp descent (before attach()) - otherwise the goal pose reports as in
# collision with the very object being grasped (see
# docs/moveit2_integration_notes.md: "Known limitation: grasp-pose collision").
GRIPPER_LINKS = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]


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
    def __init__(self, moveit_py: MoveItPy, planning_group: str = "panda_arm_hand",
                 frame_id: str = PLANNING_FRAME) -> None:
        self._moveit_py = moveit_py
        self._planning_group = planning_group
        self._frame_id = frame_id
        self._shapes: dict[str, Shape] = {}

    def add_object(self, object_id: str, shape: Shape, pose: Pose) -> None:
        self._shapes[object_id] = shape
        with self._moveit_py.get_planning_scene_monitor().read_write() as scene:
            scene.apply_collision_object(_box_collision_object(object_id, shape, pose, self._frame_id))
            scene.current_state.update()

    def update_pose(self, object_id: str, pose: Pose) -> None:
        # Re-applying a CollisionObject with the same id and operation=ADD
        # moves it in place (MoveIt merges by id) - no separate "move" verb.
        self.add_object(object_id, self._shapes[object_id], pose)

    def attach(self, object_id: str) -> None:
        with self._moveit_py.get_planning_scene_monitor().read_write() as scene:
            attached = AttachedCollisionObject()
            attached.link_name = "panda_hand"
            attached.object.id = object_id
            attached.object.operation = CollisionObject.ADD
            attached.touch_links = GRIPPER_LINKS
            scene.process_attached_collision_object(attached)
            scene.current_state.update()

    def detach(self, object_id: str, pose: Pose) -> None:
        with self._moveit_py.get_planning_scene_monitor().read_write() as scene:
            detached = AttachedCollisionObject()
            detached.link_name = "panda_hand"
            detached.object.id = object_id
            detached.object.operation = CollisionObject.REMOVE
            scene.process_attached_collision_object(detached)
            scene.current_state.update()
        # detaching drops it back into the world at the place pose
        self.update_pose(object_id, pose)

    def allow_collision(self, object_id: str) -> None:
        with self._moveit_py.get_planning_scene_monitor().read_write() as scene:
            acm = scene.allowed_collision_matrix
            for link in GRIPPER_LINKS:
                acm.set_entry(object_id, link, True)
            scene.current_state.update()

    def disallow_collision(self, object_id: str) -> None:
        with self._moveit_py.get_planning_scene_monitor().read_write() as scene:
            acm = scene.allowed_collision_matrix
            for link in GRIPPER_LINKS:
                acm.set_entry(object_id, link, False)
            scene.current_state.update()
