"""PlannerPort backed by cuRobo, running out-of-process via CuRoboBridge.

Spec: "cuRobo preferred; MoveIt2 acceptable" - this is that path, added
alongside MoveItPlannerAdapter rather than replacing it (--planner flag
picks one). MoveIt still owns the planning SCENE and execution; cuRobo only
computes the trajectory, given current joints + obstacle boxes read out of
MoveIt's own scene, so both adapters see the same world."""
from __future__ import annotations

import time

from astra_core.geometry.pose import Pose
from astra_core.ports.planner_port import FailureReason, PlannerPort, PlanResult
from astra_ros.curobo_bridge import CuRoboBridge
from astra_ros.moveit_scene_adapter import MoveItSceneAdapter
from astra_ros.tf_adapter import calibrate_gripper_orientation


class CuRoboTrajectory:
    """Opaque to the skill layer (PlanResult.trajectory: object). Consumed
    only by ROSExecutionAdapter.execute, which checks isinstance and drives
    a FollowJointTrajectory action directly - cuRobo waypoints aren't a
    moveit_py trajectory type, so moveit_py.execute() can't run them."""

    def __init__(self, joint_names: list[str], waypoints: list[list[float]], dt: float):
        self.joint_names = joint_names
        self.waypoints = waypoints
        self.dt = dt


class CuRoboPlannerAdapter(PlannerPort):
    def __init__(self, moveit_py, scene: MoveItSceneAdapter, robot,
                 arm_joint_names: list[str], tip_link: str,
                 curobo_robot_yml: str = "ur10e.yml"):
        self._moveit_py = moveit_py
        self._scene = scene
        self._robot = robot
        self._arm_joint_names = arm_joint_names
        self._tip_link = tip_link
        self._bridge = CuRoboBridge(robot=curobo_robot_yml)

    def _current_joint_positions(self) -> list[float]:
        """Values in self._bridge.joint_names order (whatever RobotBuilder
        found active in the URDF - includes the gripper knuckle, not just the
        arm). Reads whole-robot state by name so no group-vs-cuRobo joint
        ordering is assumed."""
        component = self._moveit_py.get_planning_component(self._robot.arm_hand_group)
        state = component.get_start_state()
        by_name = dict(zip(state.joint_positions.keys(), state.joint_positions.values())) \
            if hasattr(state.joint_positions, "keys") else dict(state.joint_positions)
        return [float(by_name.get(name, 0.0)) for name in self._bridge.joint_names]

    def _obstacles(self) -> list[dict]:
        """Boxes cuRobo should avoid: everything MoveIt knows about except a
        carried part (MoveItSceneAdapter._poses keeps a part's source pose
        even after attach(), so without this exclusion a grasped part reports
        as a static obstacle where the gripper now is), work-holding
        structure the arm has been exempted against (see
        MoveItSceneAdapter.allow_arm_collision), and a part the gripper is
        permanently exempt against post-place (allow_collision) - cuRobo has
        no per-link ACM, so the closest mirror of "gripper may touch this" is
        excluding it from the whole-body obstacle list."""
        skip = (set(self._scene.attached_ids())
                | self._scene._arm_exempt_obstacles
                | self._scene._gripper_exempt_obstacles)
        out = []
        for object_id, shape in self._scene._shapes.items():
            if object_id in skip:
                continue
            pose = self._scene._poses.get(object_id)
            if pose is None:
                continue
            qx, qy, qz, qw = (float(v) for v in pose.quat_xyzw)
            out.append({
                "name": object_id,
                "dims": [float(v) for v in shape.size],
                "pose": [float(pose.xyz[0]), float(pose.xyz[1]), float(pose.xyz[2]),
                        qw, qx, qy, qz],
            })
        return out

    def plan_to_pose(self, target: Pose, seed: int = 0) -> PlanResult:
        goal = calibrate_gripper_orientation(target, self._robot.grasp_base_rpy)
        qx, qy, qz, qw = (float(v) for v in goal.quat_xyzw)

        t0 = time.monotonic()
        response = self._bridge.plan(
            xyz=[float(v) for v in goal.xyz],
            quat_wxyz=[qw, qx, qy, qz],
            start_positions=self._current_joint_positions(),
            obstacles=self._obstacles(),
        )
        plan_ms = (time.monotonic() - t0) * 1000.0

        if not response.get("ok"):
            reason = response.get("reason", "")
            failure = FailureReason.IK_UNREACHABLE if "no solution" in reason \
                else FailureReason.PLANNING_FAILED
            return PlanResult.failed(failure, plan_time_ms=plan_ms)

        traj = CuRoboTrajectory(
            joint_names=response["joints"], waypoints=response["trajectory"],
            dt=response["dt"],
        )
        return PlanResult.ok(trajectory=traj, plan_time_ms=plan_ms)

    def check_ik(self, target: Pose) -> bool:
        goal = calibrate_gripper_orientation(target, self._robot.grasp_base_rpy)
        qx, qy, qz, qw = (float(v) for v in goal.quat_xyzw)
        response = self._bridge.plan(
            xyz=[float(v) for v in goal.xyz], quat_wxyz=[qw, qx, qy, qz],
            start_positions=self._current_joint_positions(), obstacles=self._obstacles(),
        )
        return bool(response.get("ok"))
