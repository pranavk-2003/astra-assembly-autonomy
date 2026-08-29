# MoveIt2 integration notes (M3)

## What works, verified live

`ros2 launch launch/demo.launch.py recipe:=recipes/ASTRA_Pranav_Variant_A.json` brings up the full
stack (`robot_state_publisher`, `ros2_control_node` with `mock_components` hardware, the
`joint_state_broadcaster`/`panda_arm_controller`/`panda_hand_controller` spawners, RViz) and our
`astra_ros.nodes.run_job` node, which constructs a real `MoveItPy` (OMPL pipeline) and runs the
**exact same** `astra_core.orchestrator.job_runner.run_job` used by the headless CLI - only the
three ports (`MoveItPlannerAdapter`, `MoveItSceneAdapter`, `ROSExecutionAdapter`) differ from the
mock ones.

Captured evidence: `docs/variant_a_live_launch.log` and `logs/ASTRA_PRANAV_A.jsonl`. In that run:

1. The real OMPL planner attempted `member_A`'s pick-approach pose and failed with
   `GOAL_STATE_INVALID` (see "Known limitation" below for why).
2. `MoveItPlannerAdapter` correctly mapped that to `FailureReason.PLANNING_FAILED` ->
   `FailureClass.PLANNING_COLLISION`.
3. The **unmodified** recovery policy drove the exact designed sequence - `REPLAN` (attempt 1) ->
   `REPLAN` (attempt 2, still `GOAL_STATE_INVALID`) -> `SAFE_POSE` (attempt 3) -> `OPERATOR_PAUSE`
   (terminal) - logged with `failure_class`/`action`/`attempt` at every step.
4. The job ended deterministically at `operator_paused`, matching the mock-planner test suite's
   behavior for the same failure class.

This is the core M3 claim proven: **the orchestrator, skill layer and recovery policy are
planner-independent in fact, not just in principle** - they ran, unchanged, against a real
collision-aware planner instead of `astra_sim`'s mocks.

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

## Known limitation: grasp-orientation reachability

The captured run's planning failure (`GOAL_STATE_INVALID` / "Unable to sample any valid states for
goal tree") is a genuine IK-reachability issue, not a bug in the orchestrator or adapters. Per the
spec's pose-derivation rule, `derive_grasp_pose`/`derive_approach_pose` inherit the **part's own**
orientation (the recipe's `rpy`) for the gripper target pose. For `member_A` in Variant A that
orientation is close to identity - which does not correspond to any orientation the Panda's wrist
(`panda_link8`) can reach at that position, since the recipe's `rpy` describes the *part's* pose in
the world, not a pre-computed gripper-tool-frame orientation.

**Next step (not yet implemented):** compute the grasp/approach orientation by aligning the
gripper's approach axis (its local +Z, per MoveIt2/URDF convention) with the recipe's
`approach_vector`, rather than copying the part's `rpy` directly - a small addition to
`astra_core.geometry.approach.derive_grasp_pose`/`derive_approach_pose` (an axis-alignment rotation,
composed with the existing offset math), gated behind a flag so the existing pose-math tests
(which assert the current "same orientation as target" behavior) keep passing unchanged. This does
not touch the orchestrator, skill layer, ports, or recovery policy - confirming the M3 claim above:
the parts that ARE planner-independent needed no changes to run live; the one gap found lives
entirely in the geometry layer.

## Bugs fixed in this repo's own code during live testing

- `launch/demo.launch.py`: `additional_env={"PYTHONPATH": ...}` was **replacing** the ROS-sourced
  `PYTHONPATH` for our own node's subprocess, deleting `rclpy` from it. Fixed to prepend instead.
- `src/astra_ros/nodes/run_job.py`: `--correction` was declared `type=Path` with `default=None`;
  since the launch file always passes `--correction` (empty string when none is supplied),
  `Path("")` normalizes to `Path(".")`, which is truthy - the `if args.correction` guard silently
  tried to load the current directory as a correction file. Fixed to parse as `str` first and only
  convert to `Path` once confirmed non-empty.
