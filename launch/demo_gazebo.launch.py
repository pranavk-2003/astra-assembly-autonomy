"""Gazebo (gz_sim) variant of the MoveIt2 demo: real physics (gravity,
contact) instead of ros2_control's mock_components fake hardware used by
demo.launch.py. Spawns the Panda into an empty gz_sim world via
config/panda_gazebo.urdf.xacro (this repo's own - the upstream
moveit_resources_panda_moveit_config package only ships mock_components/
isaac hardware types, not gz_sim), bridges /clock so ROS and Gazebo share a
simulated clock, then runs the identical controller-spawn-then-run_job
sequence as demo.launch.py.

Usage:
  ros2 launch launch/demo_gazebo.launch.py recipe:=recipes/ASTRA_Pranav_Variant_A.json
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

    # moveit_cpp still needs the moveit-side config (planning pipelines,
    # kinematics) - only the <ros2_control>/hardware side differs from
    # demo.launch.py, so robot_description comes from OUR xacro instead.
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path=os.path.join(REPO_ROOT, "config", "panda_gazebo.urdf.xacro"))
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(file_path=os.path.join(REPO_ROOT, "config", "moveit_cpp.yaml"))
        .to_moveit_configs()
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
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
        arguments=["-topic", "robot_description", "-name", "panda", "-z", "0.001"],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # See src/astra_ros/nodes/joint_state_restamp.py and
    # docs/moveit2_integration_notes.md: works around moveit2#2940 by
    # republishing /joint_states with a wall-clock stamp on /joint_states_wall,
    # which config/moveit_cpp_gazebo.yaml's joint_state_topic points at.
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
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )
    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )
    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("moveit_resources_panda_moveit_config"), "launch", "moveit.rviz"]
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
            # moveit_cpp_gazebo.yaml (not the default moveit_cpp.yaml) works
            # around gz_ros2_control stamping /joint_states with simulated
            # time while this node cannot be given use_sim_time (rclcpp
            # throws on the /clock QoS-override parameter - see
            # docs/moveit2_integration_notes.md and
            # github.com/moveit/moveit2/issues/2940).
            "--moveit-cpp-config", os.path.join(REPO_ROOT, "config", "moveit_cpp_gazebo.yaml"),
        ],
        cwd=REPO_ROOT,
        additional_env={
            "PYTHONPATH": os.path.join(REPO_ROOT, "src") + os.pathsep + os.environ.get("PYTHONPATH", "")
        },
        output="screen",
    )

    # Controllers only become available once gz_ros2_control's plugin has
    # initialized inside the running Gazebo process, which only happens
    # after the robot is actually spawned - chain spawn -> controllers ->
    # run_job the same deterministic way as demo.launch.py's controller race
    # fix (see docs/moveit2_integration_notes.md).
    controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner, panda_arm_controller_spawner, panda_hand_controller_spawner],
        )
    )
    run_job_after_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=panda_arm_controller_spawner,
            on_exit=[run_job_node],
        )
    )

    return LaunchDescription(
        [
            recipe_arg,
            correction_arg,
            rviz_arg,
            gz_sim,
            static_tf_node,
            robot_state_publisher,
            clock_bridge,
            joint_state_restamp_node,
            spawn_robot,
            rviz_node,
            controllers_after_spawn,
            run_job_after_controllers,
        ]
    )
