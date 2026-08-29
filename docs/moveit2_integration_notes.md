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

**Known limitation, not yet fixed:** the calibration uses a *fixed* down-facing orientation and
ignores the part's own yaw (rotation about the now-vertical approach axis). Every supplied recipe
uses `approach_vector: [0, 0, -1]` uniformly, so this is sufficient to demonstrate collision-free
planning, but a production version would compose the down-facing base rotation with the part's yaw
so the gripper's finger orientation still matches the part (documented in
`tf_adapter.py`'s docstring).

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

## Known limitation: retreat collides the just-grasped part with `keepout`

With the grasp-orientation fix (fixed down-facing quaternion, ignoring the part's own yaw) and the
allow-collision fix both in place, `Approach` and `Pick` both succeed, but the immediately following
`Retreat` (lifting the now-attached part back to the standoff) fails at the *start-state* collision
check: `'1 contact(s) detected : keepout - member_A'`. This is a direct, concrete consequence of the
grasp-orientation fix's already-documented limitation (see above): forcing every grasp to a fixed
down-facing orientation, instead of preserving the part's own yaw about the vertical approach axis,
means the grasped part is now held pointing in a fixed direction rather than its natural one - and
for `member_A` (a 0.34 m bar), that fixed direction happens to sweep into the recipe's own `keepout`
exclusion zone. Reproduced identically for both variants (same code, different recipe geometry,
same failure signature) - see `docs/variant_a_live_launch.log` and `docs/variant_b_live_launch.log`.

**Next step (not yet implemented):** the fix already scoped above - compose the down-facing base
rotation with the part's own yaw (rotation about the now-vertical approach axis) in
`tf_adapter.calibrate_gripper_orientation`, so the carried part's long axis matches its recipe
orientation instead of a fixed one. This remains confined to the adapter layer.

## Bugs fixed in this repo's own code during live testing

- `launch/demo.launch.py`: `additional_env={"PYTHONPATH": ...}` was **replacing** the ROS-sourced
  `PYTHONPATH` for our own node's subprocess, deleting `rclpy` from it. Fixed to prepend instead.
- `src/astra_ros/nodes/run_job.py`: `--correction` was declared `type=Path` with `default=None`;
  since the launch file always passes `--correction` (empty string when none is supplied),
  `Path("")` normalizes to `Path(".")`, which is truthy - the `if args.correction` guard silently
  tried to load the current directory as a correction file. Fixed to parse as `str` first and only
  convert to `Path` once confirmed non-empty.
