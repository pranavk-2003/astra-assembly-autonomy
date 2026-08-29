"""Workaround for github.com/moveit/moveit2/issues/2940 (closed as not
planned): MoveItPy cannot be given use_sim_time without rclcpp throwing on
the /clock QoS-override parameter, yet gz_ros2_control stamps /joint_states
with SIMULATED time. Every MoveIt freshness check that compares a message
timestamp against this node's (wall-clock) now() then fails - both
PlanningSceneMonitor's initial-state wait and TrajectoryExecutionManager's
own hardcoded 1s pre-execution check (the second is not configurable via
moveit_cpp.yaml at all).

This node subscribes /joint_states (sim-stamped, from gz_ros2_control) and
republishes the identical positions/velocities to /joint_states_wall
stamped with wall-clock now(). demo_gazebo.launch.py points
config/moveit_cpp_gazebo.yaml's joint_state_topic at /joint_states_wall
instead of /joint_states, so both of MoveIt's freshness checks see messages
whose timestamp domain matches their own clock. Confined entirely to
astra_ros/ - no astra_core or launch/demo.launch.py (mock_components) change."""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateRestamp(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_restamp")
        self._pub = self.create_publisher(JointState, "/joint_states_wall", 10)
        self._sub = self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

    def _on_joint_states(self, msg: JointState) -> None:
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = JointStateRestamp()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
