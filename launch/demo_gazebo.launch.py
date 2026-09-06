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
        .robot_description(
            file_path=os.path.join(REPO_ROOT, "config", "panda_gazebo.urdf.xacro"),
            mappings={
                "controllers_config_path": os.path.join(REPO_ROOT, "config", "ros2_controllers_gazebo.yaml"),
            },
        )
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
        launch_arguments={
            "gz_args": "-r --physics-engine gz-physics-bullet-featherstone-plugin empty.sdf"
        }.items(),
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
    # docs/moveit2_integration_notes.md: works around moveit2#2940. The xacro's
    # gz_ros2_control plugin remaps its raw output to /joint_states_raw; this
    # node republishes it wall-clock-stamped on the default /joint_states name
    # everything else (MoveItPy included) already expects.
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
            # Default moveit_cpp.yaml is fine here (same as demo.launch.py) -
            # panda_gazebo.urdf.xacro's gz_ros2_control plugin already remaps
            # its raw joint states off "/joint_states" so
            # joint_state_restamp.py can own that name with a wall-clock
            # stamp; no MoveIt-side redirect needed. See
            # docs/moveit2_integration_notes.md.
            #
            # allowed_start_tolerance widened: real physics means the arm is
            # still micro-settling from the previous motion when the next
            # trajectory starts - mock_components has no such lag, so this
            # only applies here.
            "--allowed-start-tolerance", "0.05",
            "--spawn-in-gazebo",
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
    # A spawner reporting "Configured and activated" does not guarantee the
    # controller's FollowJointTrajectory action server is registered yet
    # inside the Gazebo-embedded controller_manager - a gap the earlier
    # 10-second MoveItPy planning-scene-monitor bug was accidentally masking
    # (see docs/moveit2_integration_notes.md, "three rounds"). A plain fixed
    # delay was tried and found flaky (worked with RViz off, raced again with
    # RViz on and its extra CPU load). Polling `ros2 action list` alone is
    # ALSO flaky on its own: a fresh `ros2 action list` process completing its
    # own DDS discovery does not guarantee run_job's (about-to-start) action
    # CLIENT will have finished its own separate discovery/matching by the
    # same instant. Combine both: wait for the action to genuinely exist in
    # the graph, then a short fixed margin for the client-side race.
    wait_for_arm_controller = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "until ros2 action list 2>/dev/null | grep -q "
            "'/panda_arm_controller/follow_joint_trajectory'; do sleep 0.2; done",
        ],
        output="log",
    )
    run_job_after_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=panda_arm_controller_spawner,
            on_exit=[wait_for_arm_controller],
        )
    )
    # The action-server check above proves the CONTROLLER exists, not that
    # /joint_states (the actual dependency run_job's CurrentStateMonitor
    # needs) is flowing yet - a third, distinct instance of the same
    # DDS-discovery-timing pattern (confirmed live: MoveItPy still timed out
    # with "latest received state has time 0.000000" even after the
    # action-server check passed). Wait on that dependency directly instead
    # of a fixed margin guessing at it.
    wait_for_joint_states = ExecuteProcess(
        cmd=["bash", "-c", "timeout 15 ros2 topic echo /joint_states --once >/dev/null 2>&1"],
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
