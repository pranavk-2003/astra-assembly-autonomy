"""UR variant of the Gazebo demo - the SAME job, skills, orchestrator and
recovery as demo_gazebo.launch.py, on a different arm.

Only the robot-facing layer differs: this repo's own UR URDF (config/
ur_gazebo.urdf.xacro, which adds a gripper since UR arms ship without one)
and SRDF, the UR controllers, and `--robot ur`, which selects the naming
profile in astra_ros.robot_profile. Nothing in astra_core is aware of either
arm - that is the point of the exercise.

Usage:
  ros2 launch launch/demo_gazebo_ur.launch.py recipe:=recipes/ASTRA_Pranav_Variant_A.json
"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    recipe_arg = DeclareLaunchArgument(
        "recipe", default_value=os.path.join(REPO_ROOT, "recipes", "ASTRA_Pranav_Variant_A.json")
    )
    correction_arg = DeclareLaunchArgument("correction", default_value="")
    rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    # Overridable so an automated run can pass "-s ..." for a server-only,
    # GUI-less simulation. The GUI is by far the heaviest process here, and on
    # a loaded machine it starves the controller manager badly enough that the
    # controllers never come up ("Failed to acquire lock in 20 seconds").
    planner_arg = DeclareLaunchArgument("planner", default_value="moveit")
    gz_args_arg = DeclareLaunchArgument(
        "gz_args",
        default_value="-r --physics-engine gz-physics-bullet-featherstone-plugin empty.sdf",
    )
    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
        .robot_description(
            file_path=os.path.join(REPO_ROOT, "config", "ur_gazebo.urdf.xacro"),
            mappings={
                "controllers_config_path": os.path.join(REPO_ROOT, "config", "ros2_controllers_ur.yaml"),
            },
        )
        .robot_description_semantic(file_path=os.path.join(REPO_ROOT, "config", "ur.srdf"))
        .joint_limits(file_path=os.path.join(REPO_ROOT, "config", "ur_joint_limits.yaml"))
        .trajectory_execution(
            file_path=os.path.join(REPO_ROOT, "config", "ur_moveit_controllers.yaml"))
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .moveit_cpp(file_path=os.path.join(REPO_ROOT, "config", "moveit_cpp.yaml"))
        .to_moveit_configs()
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items(),
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0",
                   "0.0", "0.0", "world", "base_link"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description",
                   "-name", "ur", "-z", "0.001"],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    joint_state_restamp_node = ExecuteProcess(
        cmd=["python3", "-m", "astra_ros.nodes.joint_state_restamp"],
        cwd=REPO_ROOT,
        additional_env={
            "PYTHONPATH": os.path.join(REPO_ROOT, "src") + os.pathsep + os.environ.get("PYTHONPATH", "")
        },
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )
    ur_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_arm_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )
    ur_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_hand_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    run_job_node = ExecuteProcess(
        cmd=[
            "python3", "-m", "astra_ros.nodes.run_job",
            "--recipe", LaunchConfiguration("recipe"),
            "--correction", LaunchConfiguration("correction"),
            "--allowed-start-tolerance", "0.1",
            "--robot", "ur",
            "--planner", LaunchConfiguration("planner"),
            "--spawn-in-gazebo",
        ],
        cwd=REPO_ROOT,
        additional_env={
            "PYTHONPATH": os.path.join(REPO_ROOT, "src") + os.pathsep + os.environ.get("PYTHONPATH", "")
        },
        output="screen",
    )
    controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner,
                     ur_arm_controller_spawner, ur_hand_controller_spawner],
        )
    )

    wait_for_arm_controller = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "until ros2 action list 2>/dev/null | grep -q "
            "'/ur_arm_controller/follow_joint_trajectory'; do sleep 0.2; done",
        ],
        output="log",
    )
    run_job_after_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=ur_arm_controller_spawner,
            on_exit=[wait_for_arm_controller],
        )
    )
    wait_for_joint_states = ExecuteProcess(
        cmd=["bash", "-c",
             "timeout 15 ros2 topic echo /joint_states --once >/dev/null 2>&1"],
        output="log",
    )
    joint_states_after_action_server_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_arm_controller,
            on_exit=[wait_for_joint_states],
        )
    )
    run_job_after_joint_states_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_joint_states,
            on_exit=[run_job_node],
        )
    )

    return LaunchDescription(
        [
            recipe_arg,
            correction_arg,
            rviz_arg,
            gz_args_arg,
            planner_arg,
            gz_sim,
            static_tf_node,
            robot_state_publisher,
            clock_bridge,
            joint_state_restamp_node,
            spawn_robot,
            rviz_node,
            controllers_after_spawn,
            run_job_after_controllers,
            joint_states_after_action_server_ready,
            run_job_after_joint_states_ready,
        ]
    )
