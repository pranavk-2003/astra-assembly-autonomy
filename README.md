# ASTRA Assembly Autonomy

Data-driven robotic assembly autonomy layer for the ASTRA Robotics technical assessment. Variant A
and Variant B run through identical orchestrator/skill/planner code; every part, joint and obstacle
pose comes from the recipe JSON, never from source.

**Status: M0-M3 complete; both variants run the full job to completion under physics.**

```
job ASTRA_PRANAV_A: complete (13/13 steps)
job ASTRA_PRANAV_B: complete (13/13 steps)
```

Every step - pick approach, pick (grasp verified), retreat, place approach, place, retreat, for both
parts, then the joint approach - plans, executes and verifies against a live MoveIt2/OMPL planner
driving a Gazebo simulation with real gravity and contact. The **same** orchestrator, skill and
recovery code runs both variants; only the recipe JSON changes.

The pure-Python core (recipe loading, pose math, world model, skill layer, FSM orchestrator, recovery
policy, trace logging) also runs both variants headless with mock planner/execution/scene adapters -
no ROS, no simulator required. `src/astra_ros/` holds the real MoveIt2 adapters (OMPL planner,
PlanningScene, MoveItPy execution), the only place MoveIt2/ROS types appear.

Reaching a completing run meant root-causing nine separate defects, each identified from the
planner's own contact reports or from direct measurement rather than assumed - a stubbed gripper that
never commanded the hand, a physics engine that silently dropped the finger mimic constraint, a
grasp-stall abort, a gripper that was never opened, an obstacle model that made the supplied recipes
unplannable by construction, standoff distances shorter than the gripper's own fingers, and an
unbounded subprocess call that could hang the job silently. `docs/moveit2_integration_notes.md`
carries the full account with live evidence for each.

A Gazebo (`gz_sim`) physics variant (`launch/demo_gazebo.launch.py`, `config/panda_gazebo.urdf.xacro`)
now runs the same sequence under real gravity and contact, verified live for both variants with RViz
and the Gazebo GUI running together: the robot is anchored to the ground (fixing an initial "arm
falls over" bug - the upstream URDF has no real joint anchoring the base, only a MoveIt-only SRDF
virtual joint with no effect on physics). Both parts and both obstacles are spawned as real, physical
Gazebo models (`gz model --list` confirms), not just MoveIt/RViz planning-scene geometry.
`Pick`/`Place` drive the real `panda_hand_controller` (a `GripperCommand` action, wired by upstream
`gripper_moveit_controllers.yaml`) through the SRDF's own `hand` group `open`/`close` states, so the
gripper's fingers genuinely open and close in Gazebo - measured peak opening 0.0343 rad against a
0.035 rad target, closing onto a 45 mm part.

**Known limitation:** the grasp is real in MoveIt's planning scene but not yet in the physics. Under
`gz_ros2_control`'s position command interface joints are driven kinematically and ignore contact
force, so the fingers close *through* the part rather than gripping it, and the part does not
physically follow the gripper. A truly physical grip needs `gz_sim`'s `DetachableJoint` system (or an
effort interface with gravity compensation); neither is wired yet. Getting here needed
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
