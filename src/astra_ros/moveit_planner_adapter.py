"""PlannerPort implementation backed by moveit_py (MoveItPy + OMPL). The only
other file allowed to import MoveIt2 types besides moveit_scene_adapter.py -
everything above this adapter (skills, orchestrator, recovery) works only
with astra_core.ports.planner_port.PlanResult/FailureReason."""
from __future__ import annotations

import time

from moveit.planning import MoveItPy
from moveit_msgs.msg import MoveItErrorCodes

from astra_core.geometry.pose import Pose
from astra_core.ports.planner_port import FailureReason, PlannerPort, PlanResult
from astra_ros.ros_conversions import to_pose_stamped
from astra_ros.robot_profile import PANDA, RobotProfile
from astra_ros.tf_adapter import calibrate_gripper_orientation

# moveit_msgs/MoveItErrorCodes.val -> our planner-neutral FailureReason.
# Anything not listed here falls back to PLANNING_FAILED.
_ERROR_CODE_MAP = {
    MoveItErrorCodes.NO_IK_SOLUTION: FailureReason.IK_UNREACHABLE,
    MoveItErrorCodes.GOAL_IN_COLLISION: FailureReason.COLLISION,
    MoveItErrorCodes.START_STATE_IN_COLLISION: FailureReason.COLLISION,
    # GOAL_STATE_INVALID: OMPL's generic "could not sample a valid goal state"
    # - in practice (confirmed live) this fires when the goal pose is in
    # collision with the recipe's own obstacles/parts, not an IK failure.
    MoveItErrorCodes.GOAL_STATE_INVALID: FailureReason.COLLISION,
    MoveItErrorCodes.TIMED_OUT: FailureReason.PLANNING_TIMEOUT,
}


class MoveItPlannerAdapter(PlannerPort):
    def __init__(
        self,
        moveit_py: MoveItPy,
        robot: RobotProfile = PANDA,
        planning_frame: str | None = None,
    ) -> None:
        self._moveit_py = moveit_py
        self._component_name = robot.arm_group
        self._tip_link = robot.tip_link
        # Recipe "world" maps onto the robot's own planning root.
        self._planning_frame = planning_frame or robot.base_frame
        self._robot = robot

    def plan_to_pose(self, target: Pose, seed: int = 0) -> PlanResult:
        component = self._moveit_py.get_planning_component(self._component_name)
        component.set_start_state_to_current_state()
        goal = calibrate_gripper_orientation(target, self._robot.grasp_base_rpy)
        component.set_goal_state(
            pose_stamped_msg=to_pose_stamped(goal, self._planning_frame),
            pose_link=self._tip_link,
        )

        t0 = time.monotonic()
        result = component.plan()
        plan_ms = (time.monotonic() - t0) * 1000.0

        if result:
            return PlanResult.ok(trajectory=result.trajectory, plan_time_ms=plan_ms)

        reason = _ERROR_CODE_MAP.get(result.error_code.val, FailureReason.PLANNING_FAILED)
        return PlanResult.failed(reason, plan_time_ms=plan_ms)

    def check_ik(self, target: Pose) -> bool:
        component = self._moveit_py.get_planning_component(self._component_name)
        state = component.get_start_state()
        goal = calibrate_gripper_orientation(target, self._robot.grasp_base_rpy)
        return state.set_from_ik(
            self._component_name,
            to_pose_stamped(goal, self._planning_frame).pose,
            self._tip_link,
            0.5,
        )
