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
   `VERIFY` state logged `result: ok`. This is a genuine, physically-executed (in `mock_components`
   simulation) motion, not a mocked one.
2. `Pick` (the final descent onto the grasp pose) then fails with `GOAL_STATE_INVALID` - see "Known
   limitation" below; this is a different, well-understood issue from the ones fixed along the way.
3. `MoveItPlannerAdapter` mapped that to `FailureReason.COLLISION` -> `FailureClass.PLANNING_COLLISION`.
4. The **unmodified** recovery policy drove the exact designed sequence - `REPLAN` (attempt 1) ->
   `REPLAN` (attempt 2) -> `SAFE_POSE` (attempt 3) -> `OPERATOR_PAUSE` (terminal) - logged with
   `failure_class`/`action`/`attempt` at every step, identically for both variants (different
   recipe-derived `target_xyz`, same code path - R7 live).
5. The job ended deterministically at `operator_paused`, matching the mock-planner test suite's
   behavior for the same failure class.

This is the core M3 claim proven, now with an actual successful real-robot motion inside it: **the
orchestrator, skill layer and recovery policy are planner-independent in fact, not just in
principle** - they ran, unchanged, against a real collision-aware planner instead of `astra_sim`'s
mocks, executed a real trajectory, and recovered deterministically from a real planning failure.

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

## Known limitation: grasp-pose collision (the final descent onto the part)

With both fixes above in place, `Approach` succeeds but the next step, `Pick` (descending from the
standoff to the actual grasp pose, which necessarily puts the gripper around/overlapping the part),
fails with the same `GOAL_STATE_INVALID`/collision code - because MoveIt2 has no reason yet to allow
the gripper to overlap the part it is about to grasp. This is a well-known, standard part of MoveIt2
pick-and-place (it's exactly what MoveIt Task Constructor's `ModifyPlanningScene` "allow collision"
stage exists for): before planning the final grasp descent, the object-to-be-grasped needs to be
temporarily marked as an allowed collision with the gripper links, then that allowance revoked (or
made permanent as "attached") once the grasp is confirmed.

**Next step (not yet implemented):** extend `ScenePort` with an `allow_collision(object_id, link_names)`
call (or fold it into the existing `attach()` semantics, called slightly earlier - before the final
`Pick` plan rather than after execution), implemented in `MoveItSceneAdapter` via
`AllowedCollisionMatrix` entries on the `PlanningScene`. This is a `ScenePort`/adapter-level change,
not an orchestrator, skill, or recovery-policy change - the same pattern as the two fixes above.

## Bugs fixed in this repo's own code during live testing

- `launch/demo.launch.py`: `additional_env={"PYTHONPATH": ...}` was **replacing** the ROS-sourced
  `PYTHONPATH` for our own node's subprocess, deleting `rclpy` from it. Fixed to prepend instead.
- `src/astra_ros/nodes/run_job.py`: `--correction` was declared `type=Path` with `default=None`;
  since the launch file always passes `--correction` (empty string when none is supplied),
  `Path("")` normalizes to `Path(".")`, which is truthy - the `if args.correction` guard silently
  tried to load the current directory as a correction file. Fixed to parse as `str` first and only
  convert to `Path` once confirmed non-empty.
