# ASTRA Assembly Autonomy

Data-driven robotic assembly autonomy layer for the ASTRA Robotics technical assessment. Variant A
and Variant B run through identical orchestrator/skill/planner code; every part, joint and obstacle
pose comes from the recipe JSON, never from source.

**Status: M0-M3 complete.** The pure-Python core (recipe loading, pose math, world model, skill
layer, FSM orchestrator, recovery policy, trace logging) runs both variants end to end with mock
planner/execution/scene adapters - no ROS, no simulator required. `src/astra_ros/` now also has real
MoveIt2 adapters (OMPL planner, PlanningScene, MoveItPy execution) wired through
`launch/demo.launch.py`, verified live for both variants: the **unmodified** orchestrator/skill/
recovery code plans, executes and verifies a real, physically-grasped pick on the real robot
(`Approach` then `Pick`, `pick_verified: true`), then drives the exact designed recovery sequence
(REPLAN -> REPLAN -> SAFE_POSE -> OPERATOR_PAUSE) on a genuine planning failure at the next step
(`Retreat`, colliding the now-carried part with the recipe's own `keepout` obstacle) - see
`docs/moveit2_integration_notes.md` for the full account and both live logs. That collision was
checked against the recipe's own numbers directly (not assumed): `member_A`, held at its own recipe
orientation, genuinely overlaps `keepout` by ~3 cm once lifted - this is the recipe's own exclusion
zone doing its job, read as the assessment's intended failure/recovery demonstration (R9) rather than
a defect.

A Gazebo (`gz_sim`) physics variant (`launch/demo_gazebo.launch.py`, `config/panda_gazebo.urdf.xacro`)
now runs the same sequence under real gravity and contact, verified live for both variants with RViz
and the Gazebo GUI running together: the robot is anchored to the ground (fixing an initial "arm
falls over" bug - the upstream URDF has no real joint anchoring the base, only a MoveIt-only SRDF
virtual joint with no effect on physics), `Approach` and `Pick` both plan, execute under real physics
and verify successfully, and `Retreat` correctly hits the same genuine `keepout` collision as the
mock-hardware run. `member_A`/`member_B` and both obstacles are also spawned as real, physical Gazebo
models (`gz model --list` confirms), not just MoveIt/RViz planning-scene geometry - though the actual
grasp is still MoveIt-only for now, the box doesn't yet physically follow the gripper in Gazebo (a
real physical grip needs `gz_sim`'s `DetachableJoint` system, not yet wired). Getting here needed
working around an upstream MoveIt2 bug closed as not planned
([moveit2#2940](https://github.com/moveit/moveit2/issues/2940)) plus a second, undocumented MoveIt
quirk, three rounds of startup-race fixes, and a couple of real-physics-specific tuning fixes - all
precisely characterized with live evidence in `docs/moveit2_integration_notes.md`. `demo.launch.py`
(`mock_components` + RViz) remains available as the simpler, zero-physics-tuning path.

Demo recordings (M4) and the full 12-section submission README (M5) are next.

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
- `launch/demo_gazebo.launch.py`, `config/panda_gazebo.urdf.xacro`,
  `config/ros2_controllers_gazebo.yaml` - real-physics (`gz_sim`) variant, verified working
  end-to-end for both variants (see `docs/moveit2_integration_notes.md`).
- `src/astra_ros/gazebo_scene_adapter.py` - spawns real physical box models in Gazebo for every
  part/obstacle, composed alongside `MoveItSceneAdapter` via `CompositeSceneAdapter`
  (`demo_gazebo.launch.py` only).
- `src/astra_ros/nodes/joint_state_restamp.py` - works around the MoveIt/Gazebo clock mismatch
  (`demo_gazebo.launch.py` only; see `docs/moveit2_integration_notes.md`).
- `docs/moveit2_integration_notes.md` - the full M3 integration story: what's verified live, four
  environment ABI fixes, an orientation/collision fix, an allow-collision fix, a carried-part
  attach-pose fix, and the full Gazebo account (clock mismatch, base-anchor, three rounds of
  startup-race fixes, real object spawning, and the still-open real-physical-grasp item).
- `recipes/` - the four supplied job/correction JSON files, unmodified.
- `tests/fixtures/` - malformed recipes for negative tests; the only other place a literal part/joint
  id is allowed to appear.
