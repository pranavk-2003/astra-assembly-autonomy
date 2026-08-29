# ASTRA Assembly Autonomy

Data-driven robotic assembly autonomy layer for the ASTRA Robotics technical assessment. Variant A
and Variant B run through identical orchestrator/skill/planner code; every part, joint and obstacle
pose comes from the recipe JSON, never from source.

**Status: M0-M3 complete.** The pure-Python core (recipe loading, pose math, world model, skill
layer, FSM orchestrator, recovery policy, trace logging) runs both variants end to end with mock
planner/execution/scene adapters - no ROS, no simulator required. `src/astra_ros/` now also has real
MoveIt2 adapters (OMPL planner, PlanningScene, MoveItPy execution) wired through
`launch/demo.launch.py`, verified live for both variants: the **unmodified** orchestrator/skill/
recovery code plans, executes and verifies a real collision-aware motion on the real robot
(`Approach`), then drives the exact designed recovery sequence
(REPLAN -> REPLAN -> SAFE_POSE -> OPERATOR_PAUSE) on a genuine planning failure at the next step
(`Pick`) - see `docs/moveit2_integration_notes.md` for the full account, both live logs, and one
open limitation (allowing gripper/part collision for the final grasp descent - a standard MoveIt2
pick-and-place step, documented there with its fix). Demo recordings (M4) and the full 12-section
submission README (M5) are next.

## Quick start

```bash
pip install -e ".[dev]"
pytest -q                                    # 31 tests, no ROS required

python -m astra_core.cli --recipe recipes/ASTRA_Pranav_Variant_A.json
python -m astra_core.cli --recipe recipes/ASTRA_Pranav_Variant_B.json
python -m astra_core.cli --recipe recipes/ASTRA_Pranav_Variant_B.json \
  --correction recipes/ASTRA_Pranav_Perception_Correction.json      # accepted, re-plans
python -m astra_core.cli --recipe recipes/ASTRA_Pranav_Variant_B.json \
  --correction recipes/ASTRA_Pranav_Failure_Injection.json          # rejected, operator pause

cat logs/ASTRA_PRANAV_B.jsonl | python3 -m json.tool --json-lines   # R10 trace log

# Real MoveIt2 planner (needs ROS2 Jazzy + moveit_resources_panda_moveit_config sourced):
source /opt/ros/jazzy/setup.bash
ros2 launch launch/demo.launch.py use_rviz:=false recipe:=recipes/ASTRA_Pranav_Variant_A.json
```

## Layout

- `src/astra_core/` - pure Python, zero ROS/MoveIt imports. Recipe loading & validation, pose
  math, world model, perception gate, skill layer, FSM orchestrator, recovery policy, trace logger.
- `src/astra_sim/` - headless mock Planner/Execution/Scene port implementations plus a deterministic
  `FaultInjector`, used by the CLI demo and the test suite.
- `src/astra_ros/` - the MoveIt2 adapters (planner/scene/execution) and the `run_job` node; the only
  place MoveIt2/ROS types appear.
- `launch/demo.launch.py` - full stack (robot_state_publisher, ros2_control, controller spawners,
  RViz, our node) for `ros2 launch`.
- `config/moveit_cpp.yaml` - MoveItPy-specific pipeline/scene-monitor parameters (see
  `docs/moveit2_integration_notes.md` for why this file is needed in addition to the standard
  `MoveItConfigsBuilder` chain).
- `docs/moveit2_integration_notes.md` - the M3 integration story: what's verified live, four
  environment ABI fixes, an orientation/collision fix and a launch-ordering fix made along the way,
  and the one open limitation (allowing gripper/part collision for the grasp descent).
- `recipes/` - the four supplied job/correction JSON files, unmodified.
- `tests/fixtures/` - malformed recipes for negative tests; the only other place a literal part/joint
  id is allowed to appear.
