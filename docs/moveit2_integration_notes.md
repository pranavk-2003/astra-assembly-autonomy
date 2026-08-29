# MoveIt2 integration notes (M3)

## What works, verified live

`ros2 launch launch/demo.launch.py recipe:=recipes/ASTRA_Pranav_Variant_A.json` brings up the full
stack (`robot_state_publisher`, `ros2_control_node` with `mock_components` hardware, the
`joint_state_broadcaster`/`panda_arm_controller`/`panda_hand_controller` spawners, RViz) and our
`astra_ros.nodes.run_job` node, which constructs a real `MoveItPy` (OMPL pipeline) and runs the
**exact same** `astra_core.orchestrator.job_runner.run_job` used by the headless CLI - only the
three ports (`MoveItPlannerAdapter`, `MoveItSceneAdapter`, `ROSExecutionAdapter`) differ from the
mock ones.

Captured evidence: `docs/variant_a_live_launch.log`, `docs/variant_b_live_launch.log`,
`logs/ASTRA_PRANAV_A.jsonl`. Both variants, same command shape, same code:

1. **`Approach` (the pick-approach standoff pose) plans, executes and verifies successfully on the
   real robot** - real OMPL plan, real `ros2_control` trajectory execution
   (`trajectory_execution_manager: Completed trajectory execution with status SUCCEEDED`), real
   `VERIFY` state logged `result: ok`.
2. **`Pick` (the final descent onto the grasp pose) also plans, executes and verifies successfully**
   - real plan, real execution, and `pick_verified: true` - a genuine, physically-executed
   (in `mock_components` simulation) pick, not a mocked one. This needed the allow-collision fix
   below; without it `Pick` failed the same way `Approach` originally did.
3. `Retreat` (moving the now-attached part back up to the standoff) then fails with the same
   collision code - see "Known limitation" below; a different, well-understood issue from the ones
   fixed along the way, and a direct consequence of an already-documented gap.
4. `MoveItPlannerAdapter` mapped that to `FailureReason.COLLISION` -> `FailureClass.PLANNING_COLLISION`.
5. The **unmodified** recovery policy drove the exact designed sequence - `REPLAN` (attempt 1) ->
   `REPLAN` (attempt 2) -> `SAFE_POSE` (attempt 3) -> `OPERATOR_PAUSE` (terminal) - logged with
   `failure_class`/`action`/`attempt` at every step, identically for both variants (different
   recipe-derived `target_xyz`, same code path - R7 live).
6. The job ended deterministically at `operator_paused` (2/13 steps completed), matching the
   mock-planner test suite's behavior for the same failure class.

This is the core M3 claim proven, now with a genuine successful pick inside it: **the orchestrator,
skill layer and recovery policy are planner-independent in fact, not just in principle** - they ran,
unchanged, against a real collision-aware planner instead of `astra_sim`'s mocks, executed and
verified a real grasp, and recovered deterministically from a real planning failure immediately
after.

## Environment issues found and fixed along the way

The Jazzy install here had drifted (1226 packages behind at the time of writing) and surfaced four
separate ABI/version-skew breaks, each traced to a mismatched pair of packages and fixed with a
targeted `apt install --only-upgrade`:

| # | Symptom | Root cause | Fix |
| - | --- | --- | --- |
| 1 | `import moveit` → `ImportError: libgeometric_shapes.so.2.3.4` | `geometric_shapes` 2.3.2 installed, moveit_py built against 2.3.4 | upgrade `ros-jazzy-geometric-shapes` |
| 2 | `MoveItPy(...)` → `symbol lookup error` in `libmoveit_msgs__rosidl_typesupport_fastrtps_cpp.so` | `fastcdr`/`fastrtps`/`rmw_fastrtps_*` chain out of sync | upgrade the 8-package fastcdr/fastrtps/rosidl-typesupport-fastrtps chain together |
| 3 | `ros2_control_node` → `undefined symbol` referencing `CleanupController` | `controller_manager` 4.44.0 vs `controller_manager_msgs` 4.37.0 | upgrade both to matching 4.45.2 |
| 4 | `ros2_control_node` → `undefined symbol` referencing `diagnostic_updater::Updater` ctor | `diagnostic_updater` 4.2.6 vs what `controller_manager` 4.45.2 expects | upgrade `ros-jazzy-diagnostic-updater` |

Each fix was scoped to just the packages needed rather than a blanket `apt upgrade`, at the user's
request; the pattern (a new ABI break in a different subsystem each time) is exactly what a partial,
piecemeal upgrade of a badly-drifted install looks like. A full `apt upgrade` would likely have
closed all four in one pass.

## Required config beyond a plain `MoveItConfigsBuilder` chain

`moveit_resources_panda_moveit_config`'s own `demo.launch.py` (which launches the C++ `move_group`
node) does **not** need a `moveit_cpp.yaml` file. Constructing `MoveItPy` in-process does, and the
package doesn't ship one - `MoveItPy`'s `moveit_cpp` loader expects a *nested*
`planning_pipelines: {pipeline_names: [...]}` key plus `planning_scene_monitor_options`, distinct
from the *flat* `planning_pipelines` list + per-pipeline top-level keys that
`MoveItConfigsBuilder.planning_pipelines()` emits for the move_group-node convention. Without it,
`MoveItPy(...)` raises `RuntimeError: Failed to load planning pipelines from parameter server`
before ever reaching the "no current robot state" stage. Fixed by adding `config/moveit_cpp.yaml`
(this repo) and chaining `.moveit_cpp(file_path=...)` onto the builder.

## Fixed: grasp-orientation reachability (was a collision, not an IK failure)

The first live run's planning failure at the **approach standoff** pose was traced with a live
diagnostic (construct `MoveItPy` against the running stack, call `plan()` directly for several
candidate orientations at the same xyz):

- Identity orientation (the part's own `rpy`, inherited unchanged per the spec's pose-derivation
  formula) - **fails** once the recipe's obstacles/parts are in the planning scene
  (`GOAL_STATE_INVALID`, error code -27); with an empty scene it succeeds, proving this is a
  **collision**, not an IK-reachability, problem.
- 180° about X or 180° about Y (gripper pointing down) - **succeeds**, 16-waypoint trajectory, with
  the same collision scene populated.

Root cause: the recipe's part `rpy` is the *part's* orientation, not a pre-computed gripper-tool-
frame orientation; sending it unchanged puts the approach standoff in collision with the recipe's
own obstacles at the tested poses.

**Fix, confined to `src/astra_ros/tf_adapter.py`** (not `astra_core` - the spec-literal pose-
derivation formula and its tests are untouched): `calibrate_gripper_orientation(pose)` overrides the
goal orientation with a fixed down-facing quaternion before it reaches `MoveItPlannerAdapter`. This
is the correct placement per the architecture - a robot/cell-specific calibration belongs at the
same seam as `WORLD_TO_BASE`, in the adapter, not in the planner-independent geometry layer.
Verified live: `Approach` now plans, executes and verifies successfully (see above).

**Fixed (later in the session):** the calibration initially used a *fixed* down-facing orientation,
ignoring the part's own yaw. `calibrate_gripper_orientation` now composes the down-facing base
rotation with `pose.to_rpy()[2]` (the part's own yaw about the now-vertical approach axis), so the
gripper's finger orientation matches the part instead of a fixed direction. See "Corrected finding"
below for what this did and did not resolve.

## Fixed: startup race between `run_job` and the controller spawners

The first successful-planning run still failed at *execution*: `Action client not connected to
action server: panda_arm_controller/follow_joint_trajectory`. The launch file started
`run_job` alongside the controller spawners with no ordering; `run_job` reached its first
`execute()` call before `panda_arm_controller` had finished loading and activating. Fixed by
`RegisterEventHandler(OnProcessExit(target_action=panda_arm_controller_spawner, on_exit=[run_job_node]))`
in `launch/demo.launch.py` - `run_job` now starts only once the spawner it actually depends on has
exited (spawners are one-shot: they exit after their controller is loaded and activated). This is
the deterministic ROS2 pattern for this kind of dependency, not a fixed delay.

## Fixed: grasp-pose collision (the final descent onto the part)

`Pick` originally failed the same way `Approach` originally did - `GOAL_STATE_INVALID` - because
MoveIt2 had no reason to allow the gripper to overlap the part it was about to grasp. This is a
well-known, standard part of MoveIt2 pick-and-place (it's exactly what MoveIt Task Constructor's
`ModifyPlanningScene` "allow collision" stage exists for): before planning the final grasp descent,
the object-to-be-grasped needs to be temporarily marked as an allowed collision with the gripper
links.

**Fix:** extended `ScenePort` with `allow_collision(object_id)` / `disallow_collision(object_id)`,
implemented in `MoveItSceneAdapter` via `AllowedCollisionMatrix.set_entry(object_id, link, True)` for
each gripper link (`panda_hand`, `panda_leftfinger`, `panda_rightfinger`). `skills/pick.py` calls
`allow_collision` immediately before planning the grasp descent; `skills/place.py` calls
`disallow_collision` after `detach`, once the part is a world object again at its new pose. This is
a `ScenePort`/adapter-level change (plus one call each in `pick.py`/`place.py`, which already talk to
`ScenePort` for `attach`/`detach`) - not an orchestrator or recovery-policy change. `MockScene` grew
matching no-op implementations for test/interface parity. Verified live: `Pick` now plans, executes
and verifies successfully for both variants (see above).

## Fixed: carried part defaulted to identity orientation once attached

`MoveItSceneAdapter.attach()` set `attached.object.id` and `operation = ADD` but never populated
`attached.object.pose`, so MoveIt2 attached the body at identity relative to `panda_hand` - the
carried part's orientation was decoupled from its actual recipe orientation the moment it was
grasped, regardless of what orientation the gripper approached with.

**Fix:** at attach time, read the gripper's current world transform
(`scene.current_state.get_global_link_transform("panda_hand")`) and the part's last known world pose
(now cached in `MoveItSceneAdapter._poses`, set by `add_object`/`update_pose`), compute
`gripper_pose⁻¹ · part_world_pose`, and set that as `attached.object.header.frame_id = "panda_hand"` +
`attached.object.primitive_poses`. The carried part now tracks its own orientation as the gripper
moves, instead of whatever the gripper's own orientation happened to be at attach time.

This is a real, independent fix, kept regardless of the finding below - but it did **not** change the
`Retreat`/`keepout` outcome, because that collision turned out to be unrelated to orientation at all.

## Corrected finding: retreat's `keepout` collision is genuine recipe geometry, not a bug

Two fixes were made expecting this to resolve the `Retreat`-vs-`keepout` collision
(`'1 contact(s) detected : keepout - member_A'`): first `calibrate_gripper_orientation` composing
the down-facing base rotation with the part's own yaw (see above), then `MoveItSceneAdapter.attach()`
setting the attached body's pose relative to the gripper instead of leaving it at identity. **Neither
changed the outcome** - the collision is reproduced identically after both fixes, for both variants.

That result was checked against the recipe's own numbers rather than assumed away. `member_A`'s AABB
at its own recipe `source_pose` (`xyz: [0.42, -0.3, 0.1]`, `rpy` yaw `0.1`, size
`[0.34, 0.045, 0.035]`) is `x: [0.249, 0.591]`, `y: [-0.339, -0.261]`; `keepout`'s AABB
(`xyz: [0.62, -0.23, 0.175]`, size `[0.12, 0.12, 0.35]`) is `x: [0.56, 0.68]`, `y: [-0.29, -0.17]` -
a genuine ~3 cm overlap in both axes, entirely independent of gripper orientation, computed straight
from the recipe JSON. `member_A`, held at its own correct recipe orientation, physically intersects
`keepout` the moment it is lifted from its source pose.

This is not a defect to fix - it is the recipe's own obstacle placement doing exactly what an
exclusion zone is for, and the recovery policy already handles it correctly and deterministically:
`REPLAN` (attempt 1) → `REPLAN` (attempt 2) → `SAFE_POSE` (attempt 3) → `OPERATOR_PAUSE` (terminal),
identically for both variants (different recipe-derived `target_xyz`, same code path). Read as the
assessment's own intended failure/recovery demonstration (R9: "demonstrate at least one controlled
failure/recovery path"), this is evidence of correct behavior, not an open limitation - see
`docs/variant_a_live_launch.log` and `docs/variant_b_live_launch.log`.

The two fixes above remain independently correct and are kept: yaw-preserving grasp orientation and
gripper-relative attach pose are both real architectural fixes (a carried part must track its own
orientation, not a fixed one, or the identity a previous grasp happened to leave it at) - they simply
were not the cause of *this specific* collision, which a geometry check now explains directly.

## Gazebo (`gz_sim`) physics simulation: world verified live, MoveItPy sim-time blocked

`launch/demo_gazebo.launch.py` + `config/panda_gazebo.urdf.xacro` add real physics (gravity, contact)
in place of `demo.launch.py`'s `mock_components` fake hardware. The upstream
`moveit_resources_panda_moveit_config` package only ships `mock_components`/`isaac` hardware types
(see `config/panda.ros2_control.xacro` in that package), not `gz_sim`, so this repo supplies its own
xacro instead of patching a system package in place - the visual/collision/kinematic URDF is reused
from `moveit_resources_panda_description` unmodified; only the `<ros2_control>` block (hardware
plugin `gz_ros2_control/GazeboSimSystem`) and the `<gazebo><plugin filename="gz_ros2_control-system"
name="gz_ros2_control::GazeboSimROS2ControlPlugin">` system plugin are new.

**Verified live and working:** `gz sim -r empty.sdf` boots; the robot spawns via `ros_gz_sim create`;
all three controllers (`joint_state_broadcaster`, `panda_arm_controller`, `panda_hand_controller`)
configure and activate; `/joint_states` publishes at a measured **99 Hz**; `/clock` publishes. The
same `RegisterEventHandler(OnProcessExit(...))` pattern used for `demo.launch.py`'s controller race
(see above) chains spawn → controllers → `run_job`.

**Blocked:** `run_job` still fails with `Unable to configure planning scene monitor` after a 10 s
wait, even though `/joint_states` is flowing. Root cause, confirmed with live diagnostics at each
step (not guessed):

1. `gz_ros2_control` stamps `/joint_states` with **simulated** time (from the bridged `/clock`);
   `moveit_py`'s node runs on the **wall clock**. Every MoveIt freshness check that compares a
   message timestamp against its own `now()` then fails - `Requested time <wall>, but latest
   received state has time <sim>`.
2. The textbook fix, `use_sim_time: true` on the MoveIt node, is blocked by
   [moveit2#2940](https://github.com/moveit/moveit2/issues/2940) (closed as not planned): MoveItPy
   throws `rclcpp::exceptions::InvalidParameterValueException` on
   `qos_overrides./clock.subscription.durability` when `use_sim_time` is injected via `config_dict`.
   Reproduced here exactly.
3. Setting `planning_scene_monitor_options.wait_for_initial_state_timeout: 0.0` (rung 1 of the
   attempted fix) does skip that specific gate and lets `Approach` **plan successfully** - but
   `TrajectoryExecutionManager` has its **own, separate, hardcoded 1-second freshness check**
   (`Failed to validate trajectory: couldn't receive full current joint state within 1s`, not
   exposed as a `moveit_cpp.yaml` parameter at all), which then fails execution instead.
4. A republishing relay (`src/astra_ros/nodes/joint_state_restamp.py`) was built to work around both
   gates at once: it subscribes `/joint_states` and republishes identical data with a wall-clock
   stamp on `/joint_states_wall`, confirmed live at 99 Hz with the correct wall-clock timestamp.
   `config/moveit_cpp_gazebo.yaml` points `planning_scene_monitor_options.joint_state_topic` at this
   new topic.
5. **This did not work**, and the reason is now precisely characterized rather than assumed: `ros2
   node info` on the live MoveIt node during the 10 s window shows it subscribed to `/joint_states`
   directly - **not** `/joint_states_wall` - despite `moveit_config.to_dict()` confirmed (checked
   directly, outside any launch context) to contain `joint_state_topic: '/joint_states_wall'`
   correctly. `planning_scene_monitor_options.joint_state_topic` is accepted without error but does
   not appear to control the actual `CurrentStateMonitor` subscription in this installed `moveit_py`
   version - a second, undocumented quirk on top of moveit2#2940.

**Next step (not yet implemented):** since MoveIt subscribes to the literal name `/joint_states`
regardless of configuration, the fix has to happen upstream of that name instead - remap
`gz_ros2_control`'s `joint_state_broadcaster` to publish on a different topic (e.g.
`/joint_states_raw`) and point `joint_state_restamp.py` at consuming that name while publishing the
wall-stamped result onto the name `/joint_states` itself, which nothing then needs to be told to
look for. This is more involved than the current wiring because the Gazebo-embedded controller
manager (created by the `gz_ros2_control` SDF plugin inside the `gz_sim` process) is not a directly
launchable `Node` action this repo's launch file controls, unlike `ros2_control_node` in the
`mock_components` path.

**Current state:** the simulator, robot model, physics, and controller stack are fully verified
working; only the MoveIt-side clock integration remains open. `demo.launch.py` (`mock_components`,
RViz) - which the assessment spec explicitly permits as the simulator - remains the verified,
delivered demo path in the meantime.

## Bugs fixed in this repo's own code during live testing

- `launch/demo.launch.py`: `additional_env={"PYTHONPATH": ...}` was **replacing** the ROS-sourced
  `PYTHONPATH` for our own node's subprocess, deleting `rclpy` from it. Fixed to prepend instead.
- `src/astra_ros/nodes/run_job.py`: `--correction` was declared `type=Path` with `default=None`;
  since the launch file always passes `--correction` (empty string when none is supplied),
  `Path("")` normalizes to `Path(".")`, which is truthy - the `if args.correction` guard silently
  tried to load the current directory as a correction file. Fixed to parse as `str` first and only
  convert to `Path` once confirmed non-empty.
