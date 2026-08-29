"""Workaround for github.com/moveit/moveit2/issues/2940 (closed as not
planned): MoveItPy cannot be given use_sim_time without rclcpp throwing on
the /clock QoS-override parameter, yet gz_ros2_control stamps its joint
states with SIMULATED time. Every MoveIt freshness check that compares a
message timestamp against this node's (wall-clock) now() then fails - both
PlanningSceneMonitor's initial-state wait and TrajectoryExecutionManager's
own hardcoded 1s pre-execution check (the second is not configurable via
moveit_cpp.yaml at all).

A first attempt redirected MoveIt's own joint_state_topic config to a
renamed, wall-stamped topic. Live diagnostics (ros2 node info during the
failure window) showed MoveItPy's CurrentStateMonitor subscribes to the
literal name "/joint_states" regardless of that config - a second,
undocumented quirk on top of moveit2#2940 (see
docs/moveit2_integration_notes.md).

Fixed at the source instead: config/panda_gazebo.urdf.xacro's gz_ros2_control
plugin remaps its own output off the default name onto /joint_states_raw
(sim-stamped). This node subscribes /joint_states_raw and republishes the
identical positions/velocities on the name /joint_states itself - stamped
with wall-clock now() - so every consumer (MoveIt, robot_state_publisher,
RViz) that already expects the default topic name gets wall-clock-stamped
data with no config redirect needed anywhere. Confined entirely to
astra_ros/ - no astra_core or launch/demo.launch.py (mock_components) change."""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateRestamp(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_restamp")
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self._sub = self.create_subscription(JointState, "/joint_states_raw", self._on_joint_states, 10)

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
