"""Full MoveIt2 stack for the ASTRA demo: robot_state_publisher + ros2_control
(mock hardware) + controller spawners + our run_job node (real MoveIt2
planner/scene/execution adapters, same astra_core orchestrator as the
headless CLI) + optional RViz. Adapted from
moveit_resources_panda_moveit_config's own demo.launch.py, which is the
proven-working reference for this exact package/version in this environment.

Usage:
  ros2 launch launch/demo.launch.py recipe:=recipes/ASTRA_Pranav_Variant_A.json
  ros2 launch launch/demo.launch.py recipe:=recipes/ASTRA_Pranav_Variant_B.json \
    correction:=recipes/ASTRA_Pranav_Perception_Correction.json
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={"ros2_control_hardware_type": "mock_components"},
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(file_path=os.path.join(REPO_ROOT, "config", "moveit_cpp.yaml"))
        .to_moveit_configs()
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        # identity world -> panda_link0, matching astra_ros.tf_adapter's
        # documented (currently identity) global scene transform.
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )
    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller", "-c", "/controller_manager"],
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
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    run_job_node = ExecuteProcess(
        cmd=[
            "python3", "-m", "astra_ros.nodes.run_job",
            "--recipe", LaunchConfiguration("recipe"),
            "--correction", LaunchConfiguration("correction"),
        ],
        cwd=REPO_ROOT,
        # EXTEND, not replace: rclpy et al. live on the PYTHONPATH that
        # `source /opt/ros/jazzy/setup.bash` already set - overwriting it
        # here (rather than prepending) breaks the ROS imports entirely.
        additional_env={
            "PYTHONPATH": os.path.join(REPO_ROOT, "src") + os.pathsep + os.environ.get("PYTHONPATH", "")
        },
        output="screen",
    )

    # run_job must not start until panda_arm_controller (the one it actually
    # commands) is spawned and activated - starting alongside it races the
    # controller's action server coming up and the first trajectory execution
    # aborts with "Action client not connected to action server" (confirmed
    # live; see docs/moveit2_integration_notes.md). The spawner node exits
    # once its controller is loaded+activated, so chaining on its exit is the
    # deterministic ROS2 way to sequence this - not a fixed delay.
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
            static_tf_node,
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            panda_arm_controller_spawner,
            panda_hand_controller_spawner,
            rviz_node,
            run_job_after_controllers,
        ]
    )
