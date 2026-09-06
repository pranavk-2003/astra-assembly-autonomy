# ASTRA Assembly Autonomy

Data-driven robotic assembly autonomy layer. Variant A and Variant B run through identical
orchestrator, skill and planner code; every part, joint and obstacle pose, size, approach vector and
distance comes from the recipe JSON, never from source.

Demo video: [`docs/media/final_submission.mp4`](docs/media/final_submission.mp4)

## Design

Ports and adapters. `astra_core` is pure Python with no ROS, MoveIt or cuRobo imports; everything
robot-specific sits behind a port and lives in `astra_ros`.

| Layer | Location | Responsibility |
| --- | --- | --- |
| Recipe loader | `astra_core/recipe` | Schema validation, units, frame ids; rejects malformed input |
| World model | `astra_core/world` | Collision objects, attach/detach, obstacle classification |
| Pose layer | `astra_core/geometry`, `astra_core/perception` | Approach derivation, nominal vs. observed poses |
| Skills | `astra_core/skills` | `Approach`, `Pick`, `Place`, `ApproachJoint`, `Retreat` |
| Orchestrator | `astra_core/orchestrator` | Explicit FSM with replan/retry and recovery/abort branches |
| Recovery | `astra_core/recovery` | Failure classification and a deterministic per-class policy |
| Planner adapter | `astra_ros/moveit_planner_adapter.py`, `astra_ros/curobo_planner_adapter.py` | The only place MoveIt2 / cuRobo types appear |
| Execution adapter | `astra_ros/ros_execution_adapter.py` | Trajectory handoff, feedback, gripper control |
| Scene adapters | `astra_ros/moveit_scene_adapter.py`, `astra_ros/gazebo_scene_adapter.py` | Planning scene and Gazebo physics world |

Waypoints are derived, not stored: pre-grasp, pre-place and joint-approach poses are computed as
`pose - approach_vector * approach_distance`, plus `grasp_offset_xyz` for grasps.

Robot-specific naming is centralised in `astra_ros/robot_profile.py`, so changing arms is a data
change rather than a code change.

## Running

Headless tests, no simulator required:

```bash
pytest -q
```

Simulation (ROS 2 Jazzy + Gazebo, UR5e with a Robotiq 2F-85):

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch launch/demo_gazebo_ur.launch.py \
  recipe:=recipes/ASTRA_Pranav_Variant_A.json \
  planner:=moveit
```

Swap `recipe:=` for Variant B, or `planner:=curobo` for the GPU planner. cuRobo runs
out-of-process: its virtualenv is Python 3.11 while ROS 2 Jazzy is 3.12, so `rclpy` cannot be
imported alongside it. `astra_ros/curobo_bridge.py` speaks JSON lines to
`scripts/curobo_plan_server.py` across that boundary.

The job runner can also be driven directly:

```bash
python3 -m astra_ros.nodes.run_job \
  --recipe recipes/ASTRA_Pranav_Variant_A.json \
  --robot ur --planner moveit --spawn-in-gazebo
```

## Perception gate

Corrections are gated on confidence and on the two magnitude bounds in `constraints`. Recipe `rpy`
is radians; correction `delta_rpy_deg` is degrees, converted at the boundary.

- `recipes/ASTRA_Pranav_Perception_Correction.json` is accepted: the part pose and its collision
  object are updated and dependent targets recomputed.
- `recipes/ASTRA_Pranav_Failure_Injection.json` is rejected into the deterministic
  re-observation / operator path.

## Tests

```
tests/test_recipe_validation.py      schema validation, including a rejected malformed recipe
tests/test_pose_math.py              approach-offset derivation
tests/test_correction_composition.py correction applied once, not compounded
tests/test_perception_gate.py        accept and reject cases
tests/test_variant_equivalence.py    both variants through the same code path
tests/test_fsm_behaviour.py          orchestrator state transitions and recovery
tests/test_no_hardcoded_geometry.py  scans src/ for literal part, joint and obstacle ids
```

## Layout

```
src/astra_core/    planner-independent core (no ROS imports)
src/astra_ros/     MoveIt2, cuRobo, Gazebo and execution adapters
src/astra_sim/     mock backends for headless runs
launch/            ROS 2 launch files
config/            URDF/SRDF, controllers, MoveIt and cuRobo configuration
recipes/           job recipes and perception correction messages
scripts/           smoke tests and the cuRobo planning service
docs/              integration notes, run logs and the demo video
```
