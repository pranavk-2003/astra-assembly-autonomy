#!/bin/bash
# Bring the UR cell up and drive the gripper action BY HAND, printing exactly
# what the controller reports back. Establishes what a close actually returns
# (reached_goal / stalled / position) instead of inferring it from a job that
# fails for some other reason.
exec 9>/tmp/astra_sim.lock
if ! flock -n 9; then echo "sim lock held" >&2; exit 3; fi

OUT="${1:-/tmp/gripper_probe.txt}"
kill_leftovers() {
  for pat in "[j]oint_state_restamp" "[r]obot_state_publisher" "[g]z sim" \
             "[r]un_job" "[p]arameter_bridge" "[s]tatic_transform_publisher" "[r]viz2" "[c]urobo_plan_server"; do
    pkill -9 -f "$pat" 2>/dev/null
  done
}
kill_leftovers; sleep 2
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/.."
: > "$OUT"

timeout 200 ros2 launch launch/demo_gazebo_ur.launch.py use_rviz:=false \
  gz_args:="-s -r --physics-engine gz-physics-bullet-featherstone-plugin empty.sdf" \
  recipe:=recipes/ASTRA_Pranav_Variant_A.json > /tmp/gripper_probe_launch.log 2>&1 &

until ros2 topic list 2>/dev/null | grep -q '^/joint_states$'; do sleep 1; done
sleep 12

{
  echo "=== is the action there? ==="
  ros2 action list 2>/dev/null | grep -i gripper

  echo; echo "=== action type ==="
  ros2 action info /ur_hand_controller/gripper_cmd -t 2>/dev/null

  echo; echo "=== CLOSE: position 0.7, max_effort 40 ==="
  timeout 25 ros2 action send_goal -f /ur_hand_controller/gripper_cmd \
    control_msgs/action/GripperCommand \
    "{command: {position: 0.7, max_effort: 40.0}}" 2>&1 | tail -20

  echo; echo "=== knuckle joint after close ==="
  timeout 8 ros2 topic echo /joint_states --once --field name 2>/dev/null
  timeout 8 ros2 topic echo /joint_states --once --field position 2>/dev/null
} >> "$OUT" 2>&1

kill_leftovers
echo "PROBE_DONE" >> "$OUT"
