"""Real-planner entrypoint: identical astra_core.orchestrator.job_runner.run_job
call as astra_core.cli, only the three ports differ (MoveIt2-backed instead
of mock). Proves R7/R5 against a live collision-aware planner. Must be run
via `ros2 launch astra_ros demo.launch.py` (or an equivalent launch context)
so /joint_states, robot_description, and the controller manager are already
up - MoveItPy needs a live robot_state_publisher/controller stack, it cannot
bootstrap one itself (see docs/moveit2_integration_notes.md)."""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import rclpy
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder

from astra_core.orchestrator.job_runner import run_job
from astra_core.recipe.loader import load_correction, load_recipe
from astra_core.skills.base import SkillContext
from astra_core.trace.logger import TraceLogger
from astra_ros.gazebo_scene_adapter import CompositeSceneAdapter, GazeboSceneAdapter
from astra_ros.curobo_planner_adapter import CuRoboPlannerAdapter
from astra_ros.moveit_planner_adapter import MoveItPlannerAdapter
from astra_ros.moveit_scene_adapter import MoveItSceneAdapter
from astra_ros.robot_profile import PANDA, RobotProfile, profile
from astra_ros.ros_execution_adapter import ROSExecutionAdapter


REPO_ROOT = Path(__file__).resolve().parents[3]


def _moveit_configs(robot: RobotProfile, moveit_cpp_path: str):
    """MoveIt configuration for the selected arm.

    The Panda path uses the upstream moveit_resources package as-is. The UR
    path takes kinematics/joint limits from ur_moveit_config but supplies this
    repo's own URDF and SRDF, because ur_description ships a bare arm: no
    gripper, hence no hand group and no open/close states for a grasp.
    """
    if robot.name == "ur":
        return (
            MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
            .robot_description(
                file_path=str(REPO_ROOT / "config" / "ur_gazebo.urdf.xacro"),
                mappings={
                    "controllers_config_path": str(
                        REPO_ROOT / "config" / "ros2_controllers_ur.yaml"
                    ),
                },
            )
            .robot_description_semantic(file_path=str(REPO_ROOT / "config" / "ur.srdf"))
            .joint_limits(file_path=str(REPO_ROOT / "config" / "ur_joint_limits.yaml"))
            .trajectory_execution(
                file_path=str(REPO_ROOT / "config" / "ur_moveit_controllers.yaml")
            )
            .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
            .moveit_cpp(file_path=moveit_cpp_path)
            .to_moveit_configs()
        )
    return (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(file_path=moveit_cpp_path)
        .to_moveit_configs()
    )


def build_moveit_py(
    node_name: str = "astra_run_job",
    moveit_cpp_config: str | None = None,
    allowed_start_tolerance: float | None = None,
    robot: RobotProfile = PANDA,
) -> MoveItPy:
    moveit_cpp_path = moveit_cpp_config or str(REPO_ROOT / "config" / "moveit_cpp.yaml")
    moveit_config = _moveit_configs(robot, moveit_cpp_path)
    # NOTE: use_sim_time is deliberately NOT set here. MoveItPy throws
    # rclcpp::exceptions::InvalidParameterValueException on
    # "qos_overrides./clock.subscription.durability" when use_sim_time is
    # injected via config_dict - a known, unresolved upstream bug
    # (github.com/moveit/moveit2/issues/2940, closed as not planned). The
    # sim/wall clock mismatch this causes is instead worked around at the
    # topic level (see docs/moveit2_integration_notes.md,
    # src/astra_ros/nodes/joint_state_restamp.py).
    config_dict = moveit_config.to_dict()
    if allowed_start_tolerance is not None:
        # gripper_moveit_controllers.yaml's default (0.01 rad) assumes
        # mock_components' zero-lag fake hardware. Under real physics
        # (demo_gazebo.launch.py), the arm is still micro-settling from the
        # previous motion by the time the next trajectory starts, and
        # TrajectoryExecutionManager rejects it as "start point deviates
        # from current robot state" - a real dynamics effect mock hardware
        # never exhibits, not a bug. Widened for that launch only.
        config_dict["trajectory_execution"]["allowed_start_tolerance"] = allowed_start_tolerance
        # Same real-physics-vs-planned-timing gap as above, but for
        # in-flight duration rather than start position: gz_ros_control's
        # position interface (see docs/moveit2_integration_notes.md) tracks
        # a commanded position slower than the ideal joint_limits.yaml
        # velocity assumed by AddTimeOptimalParameterization, so the
        # gripper-close trajectory's own actual completion legitimately
        # runs past the planned duration - confirmed live via "Controller is
        # taking too long to execute trajectory" on the stock 1.2/0.5
        # values. Widened for this launch only.
        config_dict["trajectory_execution"]["allowed_execution_duration_scaling"] = 3.0
        config_dict["trajectory_execution"]["allowed_goal_duration_margin"] = 2.0
    return MoveItPy(node_name=node_name, config_dict=config_dict)


def _exit(status: int) -> None:
    """Exit without running MoveItCpp's destructor.

    MoveItPy segfaults inside its own teardown ("Deleting MoveItCpp") AFTER
    the job has finished and its result has been printed. The launch then
    reports `process has died ... exit code -11`, which reads as a failed run
    even when every step succeeded. All work and logging are complete by this
    point, so the process leaves with the real status rather than reporting a
    crash it already survived.
    """
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(status)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="ASTRA MoveIt2 job runner")
    parser.add_argument("--recipe", required=True, type=Path)
    # plain str (not Path): launch always passes --correction, using "" for
    # "none supplied" - Path("") normalizes to Path(".") which is truthy, so
    # this is checked as a string before conversion.
    parser.add_argument("--correction", type=str, default="")
    parser.add_argument("--confidence-threshold", type=float, default=0.90)
    parser.add_argument("--moveit-cpp-config", type=str, default="")
    parser.add_argument("--allowed-start-tolerance", type=float, default=None)
    parser.add_argument(
        "--robot", type=str, default="panda",
        help="robot profile selecting arm/gripper naming (see astra_ros.robot_profile). "
             "Nothing in astra_core changes with it.",
    )
    parser.add_argument(
        "--planner", type=str, default="moveit", choices=["moveit", "curobo"],
        help="planner backend. cuRobo runs out-of-process (its venv is py3.11, "
             "ROS Jazzy is py3.12, rclpy can't load in cuRobo's venv) - see "
             "astra_ros.curobo_bridge. MoveIt still owns the planning scene and "
             "execution either way; cuRobo only computes trajectories.",
    )
    parser.add_argument(
        "--spawn-in-gazebo",
        action="store_true",
        help="also spawn/move real physical box models in Gazebo for every part/obstacle "
        "(demo_gazebo.launch.py only) - keeps MoveIt's own planning-scene collision "
        "model unchanged, adds a real Gazebo presence alongside it via CompositeSceneAdapter",
    )
    args = parser.parse_args(argv)

    recipe = load_recipe(args.recipe)
    correction = load_correction(Path(args.correction)) if args.correction else None

    rclpy.init()
    robot = profile(args.robot)
    moveit_py = build_moveit_py(
        moveit_cpp_config=args.moveit_cpp_config or None,
        allowed_start_tolerance=args.allowed_start_tolerance,
        robot=robot,
    )
    try:
        logger = TraceLogger(job_id=recipe.job_id, out_path=Path("logs") / f"{recipe.job_id}.jsonl")
        moveit_scene = MoveItSceneAdapter(moveit_py, robot=robot)
        scene = moveit_scene
        if args.spawn_in_gazebo:
            scene = CompositeSceneAdapter([scene, GazeboSceneAdapter()])

        if args.planner == "curobo":
            if not robot.curobo_robot_yml:
                raise SystemExit(f"--planner curobo: robot profile {robot.name!r} has no curobo_robot_yml")
            planner = CuRoboPlannerAdapter(
                moveit_py, moveit_scene, robot,
                arm_joint_names=[], tip_link=robot.tip_link,
                curobo_robot_yml=robot.curobo_robot_yml,
            )
        else:
            planner = MoveItPlannerAdapter(moveit_py, robot=robot)

        ctx = SkillContext(
            planner=planner,
            execution=ROSExecutionAdapter(moveit_py, robot=robot),
            scene=scene,
            logger=logger,
            job_id=recipe.job_id,
        )
        result = run_job(recipe, ctx, correction=correction, confidence_threshold=args.confidence_threshold)
        print(
            f"job {recipe.job_id}: {result.status.value} "
            f"({result.steps_completed}/{result.steps_total} steps)",
            file=sys.stderr,
        )
        # Leave immediately, skipping MoveItCpp teardown - see _exit(). Placed
        # inside the try so the finally below (which is what actually crashes)
        # never runs on a normal finish; it still cleans up on an exception.
        _exit(0 if result.status.value == "complete" else 1)
    finally:
        moveit_py.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    _exit(main() or 0)
