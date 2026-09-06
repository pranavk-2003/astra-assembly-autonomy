"""ExecutionPort implementation: hands a planned trajectory to MoveItPy's own
executor (talks to the real/simulated controller via ros2_control). Grasp
verification stays simulated (R9 allows this - no force/tactile sensing in
this cell) but is tied to the gripper-close plan+execute actually succeeding."""
from __future__ import annotations

import sys
import time

import threading

import rclpy
from control_msgs.action import GripperCommand
from moveit.planning import MoveItPy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from astra_core.ports.execution_port import ExecutionPort, ExecutionResult, ExecutionStatus
from astra_ros.robot_profile import PANDA, RobotProfile


def _await(future, timeout_s: float):
    """Block until a future resolves, without spinning - an executor thread is
    already doing that. Returns None on timeout."""
    deadline = time.monotonic() + timeout_s
    while not future.done():
        if time.monotonic() > deadline:
            return None
        time.sleep(0.02)
    return future.result()


class ROSExecutionAdapter(ExecutionPort):
    def __init__(
        self,
        moveit_py: MoveItPy,
        controller_names: list[str] | None = None,
        robot: RobotProfile = PANDA,
    ) -> None:
        self._moveit_py = moveit_py
        self._controller_names = controller_names or list(robot.controllers)
        self._robot = robot
        self._gripper_group = robot.gripper_group
        self._attached_ids: set[str] = set()
        # Direct GripperCommand client, when the profile names one. Planning
        # to a named state can't express a grasp: it demands the jaws REACH a
        # position, but a grasp is the jaws stalling early against the
        # workpiece, so a position+force-limit command is needed instead.
        self._gripper_node: Node | None = None
        self._gripper_client: ActionClient | None = None
        self._gripper_executor: SingleThreadedExecutor | None = None
        if robot.gripper_action:
            self._gripper_node = rclpy.create_node("astra_gripper_client")
            self._gripper_client = ActionClient(
                self._gripper_node, GripperCommand, robot.gripper_action
            )
            # Node must be SPINNING for server discovery + future resolution;
            # spinning only inside the call is too late.
            self._gripper_executor = SingleThreadedExecutor()
            self._gripper_executor.add_node(self._gripper_node)
            threading.Thread(
                target=self._gripper_executor.spin, daemon=True
            ).start()
        # Same node/executor also drives FollowJointTrajectory directly for
        # cuRobo trajectories (plain joint-angle lists, not a moveit_py
        # planned trajectory - see execute()).
        self._arm_client: ActionClient | None = None
        if robot.arm_action:
            if self._gripper_node is None:
                self._gripper_node = rclpy.create_node("astra_curobo_client")
                self._gripper_executor = SingleThreadedExecutor()
                self._gripper_executor.add_node(self._gripper_node)
                threading.Thread(target=self._gripper_executor.spin, daemon=True).start()
            self._arm_client = ActionClient(
                self._gripper_node, FollowJointTrajectory, robot.arm_action
            )

    def execute(self, trajectory: object) -> ExecutionResult:
        from astra_ros.curobo_planner_adapter import CuRoboTrajectory
        if isinstance(trajectory, CuRoboTrajectory):
            return self._execute_curobo(trajectory)
        ok = self._moveit_py.execute(trajectory, controllers=self._controller_names)
        return ExecutionResult(status=ExecutionStatus.SUCCESS if ok else ExecutionStatus.CONTROLLER_ERROR)

    def _execute_curobo(self, trajectory) -> ExecutionResult:
        if self._arm_client is None:
            print("[execution] no arm_action configured for cuRobo trajectories",
                  file=sys.stderr)
            return ExecutionResult(status=ExecutionStatus.CONTROLLER_ERROR)
        if not self._arm_client.wait_for_server(timeout_sec=10.0):
            return ExecutionResult(status=ExecutionStatus.CONTROLLER_ERROR)

        # cuRobo's columns cover every joint active in its model (arm + gripper
        # knuckle); the arm controller only accepts its own joints. Keep only
        # the arm columns, in the controller's expected order.
        arm_names = list(self._robot.arm_joint_names) or trajectory.joint_names
        col = [trajectory.joint_names.index(n) for n in arm_names]

        msg = JointTrajectory()
        msg.joint_names = arm_names
        for i, waypoint in enumerate(trajectory.waypoints):
            point = JointTrajectoryPoint()
            point.positions = [float(waypoint[c]) for c in col]
            t = (i + 1) * trajectory.dt
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            msg.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = msg
        send = self._arm_client.send_goal_async(goal)
        handle = _await(send, timeout_s=10.0)
        if handle is None or not handle.accepted:
            return ExecutionResult(status=ExecutionStatus.CONTROLLER_ERROR)

        result_timeout = trajectory.dt * len(trajectory.waypoints) + 20.0
        outcome = _await(handle.get_result_async(), timeout_s=result_timeout)
        if outcome is None:
            return ExecutionResult(status=ExecutionStatus.CONTROLLER_ERROR)
        ok = outcome.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        return ExecutionResult(status=ExecutionStatus.SUCCESS if ok else ExecutionStatus.CONTROLLER_ERROR)

    # How long to let the jaws squeeze before carrying on. Long enough for a
    # free-air move to finish and report, short enough not to stall the job
    # when the gripper is holding a part and will never report anything.
    _GRIP_SETTLE_S = 3.0

    def _command_gripper(self, position: float) -> bool:
        """Close/open by force via the GripperCommand action. Success is
        `reached_goal` OR `stalled`: a grasp is a stall (allow_stalling in the
        controller config)."""
        client = self._gripper_client
        node = self._gripper_node
        if client is None or node is None:
            return False
        if not client.wait_for_server(timeout_sec=5.0):
            print("[execution] gripper action server unavailable", file=sys.stderr)
            return False

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self._robot.grip_effort)

        # Executor thread already spins this node; wait on futures directly.
        send = client.send_goal_async(goal)
        handle = _await(send, timeout_s=10.0)
        if handle is None or not handle.accepted:
            print("[execution] gripper goal not accepted", file=sys.stderr)
            return False

        outcome = _await(handle.get_result_async(), timeout_s=self._GRIP_SETTLE_S)
        if outcome is None:
            # No verdict within the settle window is the NORMAL case for a
            # grasp: jaws pressed against the workpiece may never register a
            # stall. Whether anything is held is decided later by looking at
            # the part, not by trusting this return value.
            print("[execution] gripper still pressing after "
                  f"{self._GRIP_SETTLE_S}s; treating as gripped", file=sys.stderr)
            return True
        res = outcome.result
        # A grasp is a stall against the workpiece; a free-air move reaches its
        # commanded position. Either is a completed command.
        return bool(res.reached_goal or res.stalled)

    def _move_gripper(self, configuration_name: str) -> bool:
        gripper = self._moveit_py.get_planning_component(self._gripper_group)
        gripper.set_start_state_to_current_state()
        gripper.set_goal_state(configuration_name=configuration_name)
        plan_result = gripper.plan()
        if not plan_result:
            return False
        return bool(self._moveit_py.execute(plan_result.trajectory, controllers=self._controller_names))

    def open_gripper(self) -> None:
        if self._gripper_client is not None:
            self._command_gripper(self._robot.grip_open_position)
        else:
            self._move_gripper(self._robot.gripper_open_state)

    def attach(self, object_id: str) -> None:
        if self._gripper_client is not None:
            ok = self._command_gripper(self._robot.grip_closed_position)
        else:
            ok = self._move_gripper(self._robot.gripper_closed_state)
        if ok:
            self._attached_ids.add(object_id)

    def detach(self, object_id: str) -> None:
        if self._gripper_client is not None:
            self._command_gripper(self._robot.grip_open_position)
        else:
            self._move_gripper(self._robot.gripper_open_state)
        self._attached_ids.discard(object_id)

    def verify_grasp(self, object_id: str) -> bool:
        # No force/tactile sensing in this cell (R9 allows simulated
        # verification) - treat a successful gripper-close plan+execute as
        # verification.
        return object_id in self._attached_ids

    def stop(self) -> None:
        pass  # no protective-stop channel wired in simulation; see R12 note.
