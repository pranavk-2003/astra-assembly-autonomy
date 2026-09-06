"""Everything about a specific robot that the adapters need, in one place.

The skill layer, orchestrator and world model never name a link, joint group
or controller - they work in poses. But the MoveIt adapters necessarily do,
and those names were previously written inline, which made the arm a
compile-time decision. Collecting them here makes swapping arms a matter of
selecting a profile: nothing in astra_core changes, and the adapters change
only in that they read these fields instead of literals.

Adding a new arm means adding a profile plus its URDF/SRDF and controller
config - no edit to any adapter, skill or orchestrator.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path


@dataclass(frozen=True)
class RobotProfile:
    name: str

    base_frame: str
    """Robot's planning root. The recipe's "world" frame maps onto this (see
    astra_ros.tf_adapter), so recipe coordinates are interpreted here."""

    # --- planning ---
    arm_group: str
    """SRDF planning group for the arm alone."""

    arm_hand_group: str
    """SRDF group spanning arm + gripper, used for scene/state queries."""

    tip_link: str
    """Link the planner poses. NOTE: the frame a target pose is expressed for
    - grasp offsets in the recipe are relative to this."""

    # --- gripper ---
    gripper_group: str
    """SRDF planning group for the fingers."""

    gripper_open_state: str
    gripper_closed_state: str
    """SRDF named states. A grasp is 'plan to this named state and execute'."""

    attach_link: str
    """Link a grasped part is attached to (the palm/flange the fingers hang
    off), and the frame its carried pose is expressed in."""

    gripper_links: tuple[str, ...]
    """Links permitted to touch a part being grasped or just released. Must
    include the wrist flange the hand mounts on: a held part rests against it
    by construction, and omitting it blocks the withdrawal after a place."""

    arm_links: tuple[str, ...]
    """Every link of the kinematic chain. Exempted only against work-holding
    structure the robot must physically reach inside."""

    # --- planning ---
    planning_pipeline: str = "ompl"
    """Which configured pipeline to plan with."""

    planner_id: str = ""
    """Planner within that pipeline. Empty uses the pipeline's default."""

    # --- grasp calibration ---
    grasp_base_rpy: tuple[float, float, float] = (3.141592653589793, 0.0, 0.0)
    """Orientation that points this robot's tool frame DOWN at the work, before
    the part's own yaw is applied about that axis (see
    astra_ros.tf_adapter.calibrate_gripper_orientation).

    The roll term is the per-robot part: it decides which way the jaws close.
    Both flanges here have z out of the tool, so both need the pi flip; they
    differ by a quarter turn about that axis because the gripper is mounted at
    a different clocking, which shows up as the jaws closing along a part's
    length instead of across its width.
    """

    # --- execution ---
    arm_joint_names: tuple[str, ...] = ()
    """Arm-only joint names, in the order the arm controller expects. cuRobo's
    trajectory columns cover every active joint in its model (arm + gripper
    knuckle); this filters its output down to arm-only before sending."""

    arm_action: str = ""
    """FollowJointTrajectory action for direct (non-MoveIt-executed) arm
    control. Only cuRobo's execution path uses this - see
    ROSExecutionAdapter._execute_curobo."""

    curobo_robot_yml: str = ""
    """cuRobo robot config name (shipped, e.g. ur10e.yml). Empty = no cuRobo
    support declared for this profile."""

    gripper_action: str = ""
    """GripperCommand action to drive the jaws directly. Empty falls back to
    planning to a named state, which only works where the jaws can actually
    REACH the commanded position - i.e. never when closing on a workpiece."""

    grip_effort: float = 100.0
    """Clamping force limit for a grasp, in the joint's own units. Friction
    available to hold a part is proportional to this, so it must comfortably
    exceed what the part's weight demands."""

    grip_closed_position: float = 0.7
    """Position commanded when closing. Deliberately past where the workpiece
    stops the jaws: the stall against the part IS the grasp."""

    grip_open_position: float = 0.02
    """Position commanded when opening; kept off the joint's hard limit."""

    controllers: tuple[str, ...] = ()
    """Controller names handed to MoveIt's executor; empty means 'let MoveIt
    choose from its configured controller manager'."""


PANDA = RobotProfile(
    name="panda",
    base_frame="panda_link0",
    arm_group="panda_arm",
    arm_hand_group="panda_arm_hand",
    tip_link="panda_link8",
    gripper_group="hand",
    gripper_open_state="open",
    gripper_closed_state="close",
    attach_link="panda_hand",
    gripper_links=("panda_hand", "panda_leftfinger", "panda_rightfinger", "panda_link7"),
    arm_links=tuple(f"panda_link{i}" for i in range(8))
    + ("panda_hand", "panda_leftfinger", "panda_rightfinger"),
    grasp_base_rpy=(3.141592653589793, 0.0, 0.0),
)

# UR5e + Robotiq 2F-85, joined by robotiq_description's own UR coupling plate
# (see config/ur_gazebo.urdf.xacro) - the pairing used on real UR cells. The
# 2F-85 is linkage-driven: one actuated knuckle joint, five mimic joints, so
# the gripper group has a single active joint and open/close are angles on it
# (0.0 rad fully open at 85 mm, 0.8 rad closed).
UR = RobotProfile(
    name="ur",
    base_frame="base_link",
    arm_group="ur_manipulator",
    arm_hand_group="ur_manipulator_hand",
    # The frame a grasp pose refers to: BETWEEN the fingertips, not the tool
    # flange. Posing tool0 at the target instead buries the gripper ~0.15 m
    # into the workpiece and the table (see config/ur_gazebo.urdf.xacro).
    tip_link="grasp_tcp",
    gripper_group="hand",
    gripper_open_state="open",
    gripper_closed_state="close",
    attach_link="robotiq_85_base_link",
    gripper_links=(
        "robotiq_85_base_link",
        "robotiq_85_left_knuckle_link", "robotiq_85_right_knuckle_link",
        "robotiq_85_left_finger_link", "robotiq_85_right_finger_link",
        "robotiq_85_left_inner_knuckle_link", "robotiq_85_right_inner_knuckle_link",
        "robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link",
        # The flange the gripper bolts onto: a held part rests against it by
        # construction, exactly as panda_link7 does on the Panda.
        "ur_to_robotiq_link", "gripper_mount_link", "wrist_3_link",
    ),
    arm_links=(
        "base_link", "base_link_inertia", "shoulder_link", "upper_arm_link",
        "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link",
        "flange", "tool0", "ur_to_robotiq_link", "gripper_mount_link",
        "robotiq_85_base_link",
        "robotiq_85_left_knuckle_link", "robotiq_85_right_knuckle_link",
        "robotiq_85_left_finger_link", "robotiq_85_right_finger_link",
        "robotiq_85_left_inner_knuckle_link", "robotiq_85_right_inner_knuckle_link",
        "robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link",
    ),
    # Quarter turn relative to the Panda: without it the 2F-85's jaws close
    # along the bar's 340 mm length instead of across its 45 mm width, which is
    # both unGRASPable and visibly wrong (observed in simulation).
    grasp_base_rpy=(3.141592653589793, 0.0, 1.5707963267948966),
    # OMPL, deliberately: Pilz PTP is a direct point-to-point move with no
    # obstacle avoidance, unsuitable for a cell with exclusion zones.
    planning_pipeline="ompl",
    arm_joint_names=(
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    ),
    arm_action="/ur_arm_controller/follow_joint_trajectory",
    # Not the stock ur10e.yml (bare UR10e tool0, no gripper/TCP offset) - built
    # from our actual URDF (arm + Robotiq + grasp_tcp) via
    # curobo.robot_builder.RobotBuilder instead.
    curobo_robot_yml=str(Path(__file__).resolve().parents[2] / "config" / "curobo" / "ur5e_astra.yml"),
    gripper_action="/ur_hand_controller/gripper_cmd",
    grip_effort=100.0,
    grip_closed_position=0.7,
    grip_open_position=0.02,
)

PROFILES: dict[str, RobotProfile] = {p.name: p for p in (PANDA, UR)}


def profile(name: str) -> RobotProfile:
    try:
        return PROFILES[name]
    except KeyError:
        raise ValueError(
            f"unknown robot profile {name!r}; available: {sorted(PROFILES)}"
        ) from None
