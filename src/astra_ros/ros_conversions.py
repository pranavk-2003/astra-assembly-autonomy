"""Pose <-> geometry_msgs conversion. The only file that touches both
astra_core.geometry.Pose and a ROS message type - kept separate from the
adapters so each adapter file reads as pure MoveIt2 API usage."""
from __future__ import annotations

from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import PoseStamped

from astra_core.geometry.pose import Pose

PLANNING_FRAME = "panda_link0"  # == recipe "world" frame under the identity tf_adapter transform


def to_pose_msg(pose: Pose) -> PoseMsg:
    msg = PoseMsg()
    msg.position.x, msg.position.y, msg.position.z = (float(v) for v in pose.xyz)
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = (
        float(v) for v in pose.quat_xyzw
    )
    return msg


def to_pose_stamped(pose: Pose, frame_id: str = PLANNING_FRAME) -> PoseStamped:
    stamped = PoseStamped()
    stamped.header.frame_id = frame_id
    stamped.pose = to_pose_msg(pose)
    return stamped
