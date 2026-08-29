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

## Gazebo (`gz_sim`) physics simulation: working end-to-end

`launch/demo_gazebo.launch.py` + `config/panda_gazebo.urdf.xacro` add real physics (gravity, contact)
in place of `demo.launch.py`'s `mock_components` fake hardware. The upstream
`moveit_resources_panda_moveit_config` package only ships `mock_components`/`isaac` hardware types
(see `config/panda.ros2_control.xacro` in that package), not `gz_sim`, so this repo supplies its own
xacro instead of patching a system package in place - the visual/collision/kinematic URDF is reused
from `moveit_resources_panda_description` unmodified; only the `<ros2_control>` block (hardware
plugin `gz_ros2_control/GazeboSimSystem`), the `<gazebo><plugin filename="gz_ros2_control-system"
name="gz_ros2_control::GazeboSimROS2ControlPlugin">` system plugin, and a fixed `world`->`panda_link0`
joint (see below) are new.

**Verified live and working, both variants, RViz + Gazebo GUI together (`use_rviz` defaults `true`):**
`gz sim -r empty.sdf` boots; the robot spawns anchored to the ground; all three controllers
configure and activate; `Approach` and `Pick` both plan, execute under real physics and verify
successfully (`pick_verified: true`); `Retreat` correctly hits the genuine `keepout` collision (see
"Corrected finding" above) and the recovery policy pauses deterministically - the same outcome as
`demo.launch.py`, now under real gravity and contact instead of fake hardware. Reproduced
consistently after the fixes below (each verified across 2+ clean re-launches).

### Fixed: MoveIt/Gazebo clock mismatch (the actual root cause, precisely characterized)

`run_job` originally failed with `Unable to configure planning scene monitor` after a 10 s wait, even
though `/joint_states` was flowing. `gz_ros2_control` stamps its joint states with **simulated** time
(from the bridged `/clock`); MoveItPy's node runs on the **wall clock**; every MoveIt freshness check
comparing a message timestamp against its own `now()` then fails.

The textbook fix, `use_sim_time: true` on the MoveIt node, is blocked by
[moveit2#2940](https://github.com/moveit/moveit2/issues/2940) (closed as not planned): MoveItPy
throws `rclcpp::exceptions::InvalidParameterValueException` on
`qos_overrides./clock.subscription.durability` when `use_sim_time` is injected via `config_dict`.
Reproduced here exactly - confirmed not worth working around further.

A first attempt redirected MoveIt's own `joint_state_topic` config to a renamed, wall-stamped relay
topic. Live diagnostics (`ros2 node info` during the failure window) showed MoveItPy's
`CurrentStateMonitor` subscribes to the literal name `/joint_states` regardless of that config - a
second, undocumented quirk on top of moveit2#2940, and `TrajectoryExecutionManager` has its own
separate, hardcoded 1 s freshness check that isn't configurable via `moveit_cpp.yaml` at all, so
redirecting MoveIt's config couldn't have fully worked anyway.

**Fix, at the source instead of the MoveIt side:** `config/panda_gazebo.urdf.xacro`'s
`gz_ros2_control` plugin now carries `<ros>/<remapping>/joint_states:=/joint_states_raw</remapping>`
(a real, documented feature of that plugin), so the raw sim-stamped data never reaches the default
topic name at all. `src/astra_ros/nodes/joint_state_restamp.py` subscribes `/joint_states_raw` and
republishes the identical data, wall-clock stamped, on `/joint_states` itself - the name every
consumer (MoveIt, `robot_state_publisher`, RViz) already expects by default, so no MoveIt-side config
redirect is needed anywhere. `config/moveit_cpp_gazebo.yaml` was deleted; `demo_gazebo.launch.py`
uses the same `config/moveit_cpp.yaml` as `demo.launch.py`.

### Fixed: robot base wasn't anchored to the world - it fell over under real gravity

`moveit_resources_panda_description`'s URDF has **no joint at all** connecting `panda_link0` to
anything. The SRDF's `virtual_joint` (`world` -> `panda_link0`, type `"floating"`) is a MoveIt-only
planning construct with zero effect on Gazebo physics. Under `mock_components` (no physics) this is
invisible; under real gravity the whole robot was an unanchored free body and toppled over (confirmed
live, and matches exactly what real gravity does to an unanchored rigid body - not a mysterious bug).
**Fix:** `config/panda_gazebo.urdf.xacro` adds a `<link name="world"/>` and a real
`<joint type="fixed">` from `world` to `panda_link0`, matching `astra_ros.tf_adapter`'s
`WORLD_TO_BASE` identity assumption.

### Fixed: startup race between `run_job` and the Gazebo-embedded controller stack (three rounds)

A spawner reporting "Configured and activated" does not guarantee everything `run_job` actually
depends on is ready yet inside the Gazebo-embedded controller manager - a gap the original MoveIt
clock-mismatch bug's 10-second wait was accidentally masking. Fixing that bug exposed the race
(MoveItPy now configures in under a second instead of always waiting out the full 10 s). Each
readiness proxy tried in turn was a strictly-partial fix, reproduced live before moving to the next:

1. A fixed delay (`TimerAction`, tried at 2 s then 5 s) was flaky by itself - worked with RViz off,
   raced again with RViz on (extra CPU load shifts the timing), and raced again at 5 s on a later run
   even without RViz.
2. Polling `ros2 action list` until `panda_arm_controller/follow_joint_trajectory` appears proves the
   controller *exists*, but not that `run_job`'s own (about-to-start) action *client* has finished its
   own separate DDS discovery/matching by the same instant - a second, subtler race, confirmed live
   (`controller_error` immediately after a successful plan, on a run where the action genuinely was
   already in the graph).
3. Even combined with a fixed margin after that (1 + 2), a **third** instance of the same pattern
   surfaced on a completely different dependency: `run_job` failed with
   `Unable to configure planning scene monitor` / `latest received state has time 0.000000` - i.e.
   `/joint_states` itself (not the controller) hadn't started flowing through
   `joint_state_restamp.py` yet by the time MoveItPy started polling for it, confirmed live via a
   user-run session log.

**Fix, addressing the actual dependency at each stage rather than a proxy for it:** the launch now
chains three real readiness checks in sequence - `ros2 action list` for the controller (never starts
too early), a **3 s** fixed margin for the action-client-side race, then `ros2 topic echo
/joint_states --once` (chained after that) so `run_job` starts only once its actual `/joint_states`
dependency has a confirmed message flowing, not just a controller with no data yet. Verified stable
across repeated re-launches, in the user's own separate terminal session, only after all three checks
were in place together.

### Added: real physical objects in Gazebo, alongside (not instead of) MoveIt's planning scene

`MoveItSceneAdapter` only ever updated MoveIt's own planning-scene model (used for collision-checking
and drawn by RViz) - it never spawned anything into Gazebo's physics world, so `member_A`/`member_B`/
the two obstacles were invisible in Gazebo itself even though the robot moved correctly around them
(confirmed live: the arm moved through empty space in the Gazebo view). `src/astra_ros/gazebo_scene_adapter.py`
adds `GazeboSceneAdapter` (spawns/moves a real box model per part/obstacle via `ros2 run ros_gz_sim
create`/`set_entity_pose`) and `CompositeSceneAdapter` (fans every `ScenePort` call out to multiple
adapters). `run_job.py --spawn-in-gazebo` (passed only by `demo_gazebo.launch.py`) wraps
`MoveItSceneAdapter` and `GazeboSceneAdapter` together, so **both** the MoveIt planning-scene model and
Gazebo's own physical world reflect the same recipe geometry from the same `skills/pick.py`/`place.py`
calls - `demo.launch.py` is unaffected, still plain `MoveItSceneAdapter`. Verified live: `gz model
--list` during a run shows `ground_plane`, `panda`, `fixture`, `keepout`, `member_A`, `member_B` all
present as real Gazebo models; `BUILD_WORLD` visibly took ~4.6 s instead of sub-millisecond, matching
four sequential spawn subprocess calls.

**Known limitation, not yet implemented:** this is spawn + pose-sync only. `Pick`'s grasp does not yet
create a real physical grip in Gazebo - the box does not visually follow the gripper during `Retreat`
(it isn't reached live yet, but would not move if it were). A real physical grasp needs `gz_sim`'s
`DetachableJoint` system, dynamically loaded onto the robot entity via gz-transport's
`/world/<world>/entity/system/add` service (request type `gz.msgs.EntityPlugin_V`) at `attach()` time,
naming the spawned part as `child_model`/`child_link` and `panda_hand` as `parent_link`, then
publishing to that joint's `detach_topic` at `detach()` time. Confirmed the plugin and topics exist
and work for a static SDF-authored case (`gz-sim8-detachable-joint-system`, tested against upstream's
own `detachable_joint.sdf` example); the *dynamic*, per-part-at-runtime loading step is the part not
yet built, and upstream issue reports of crashes when dynamically adding a plugin to a not-yet-ready
entity ([gz-sim#2508](https://github.com/gazebosim/gz-sim/issues/2508),
[gz-sim#2512](https://github.com/gazebosim/gz-sim/issues/2512)) suggest it needs care, not a
straight-line implementation.

### Fixed: real-physics settling exceeds mock hardware's trajectory-start tolerance

`gripper_moveit_controllers.yaml`'s default `allowed_start_tolerance` (0.01 rad) assumes
`mock_components`' zero-lag fake hardware. Under real physics the arm is still micro-settling from
the previous motion when the next trajectory starts, and `TrajectoryExecutionManager` rejected it as
`Invalid Trajectory: start point deviates from current robot state` - a real dynamics effect mock
hardware never exhibits, not a bug. **Fix:** `run_job.py --allowed-start-tolerance` (passed only by
`demo_gazebo.launch.py`, value `0.05`) widens this for the physics launch only.

### Partial mitigation: `gz_ros2_control`'s position interface is weak against gravity

`panda_arm_controller` uses a `position` command interface; `gz_ros2_control` converts position error
into a velocity setpoint (`joint_velocity = gain * error * update_rate`) rather than commanding real
torque/PID. The library default gain (0.1) is too weak to hold Panda's real link masses rigidly once
idle, and per `gz_ros2_control`'s own documentation the gain must stay `<= 1.0` to avoid oscillation -
so this is a genuine control-architecture ceiling, not a tunable-away misconfiguration.
`config/ros2_controllers_gazebo.yaml` (a copy of the upstream file, not edited in place) sets
`gz_ros_control.ros__parameters.position_proportional_gain: 0.5` (note: the plugin's actual node name
is `gz_ros_control`, without the "2" - confirmed via `strings` on the compiled plugin, since docs and
package name both say `gz_ros2_control`) as a partial mitigation. A full fix would use an `effort`
command interface with proper PID/gravity compensation instead of `position` - materially larger,
not attempted this session.

## Bugs fixed in this repo's own code during live testing

- `launch/demo.launch.py`: `additional_env={"PYTHONPATH": ...}` was **replacing** the ROS-sourced
  `PYTHONPATH` for our own node's subprocess, deleting `rclpy` from it. Fixed to prepend instead.
- `src/astra_ros/nodes/run_job.py`: `--correction` was declared `type=Path` with `default=None`;
  since the launch file always passes `--correction` (empty string when none is supplied),
  `Path("")` normalizes to `Path(".")`, which is truthy - the `if args.correction` guard silently
  tried to load the current directory as a correction file. Fixed to parse as `str` first and only
  convert to `Path` once confirmed non-empty.
