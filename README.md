# ASTRA Assembly Autonomy

Data-driven robotic assembly autonomy layer for the ASTRA Robotics technical assessment. Variant A
and Variant B run through identical orchestrator/skill/planner code; every part, joint and obstacle
pose comes from the recipe JSON, never from source.

**Status: M0-M2 complete.** The pure-Python core (recipe loading, pose math, world model, skill
layer, FSM orchestrator, recovery policy, trace logging) runs both variants end to end with mock
planner/execution/scene adapters - no ROS, no simulator required. MoveIt2 integration (M3), the
launch files and demo recordings (M4), and the full 12-section submission README (M5) are the next
milestones.

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
```

## Layout

- `src/astra_core/` - pure Python, zero ROS/MoveIt imports. Recipe loading & validation, pose
  math, world model, perception gate, skill layer, FSM orchestrator, recovery policy, trace logger.
- `src/astra_sim/` - headless mock Planner/Execution/Scene port implementations plus a deterministic
  `FaultInjector`, used by the CLI demo and the test suite.
- `src/astra_ros/` - reserved for the MoveIt2 adapters (M3); the only place MoveIt2 types will appear.
- `recipes/` - the four supplied job/correction JSON files, unmodified.
- `tests/fixtures/` - malformed recipes for negative tests; the only other place a literal part/joint
  id is allowed to appear.
