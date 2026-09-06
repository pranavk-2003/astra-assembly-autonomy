#!/bin/bash
# Measure the ACTUAL grasp error instead of eyeballing it.
#
# Samples, live and simultaneously:
#   - where the part really is in Gazebo (it is a dynamic body now, so it
#     settles to its own resting pose; the recipe pose is only nominal)
#   - where the gripper's TCP frame really is (tf: base_link -> grasp_tcp)
#
# The difference between them at the moment of the grasp is the correction to
# apply - a number, not a guess.
LOG="${1:-/tmp/astra_grasp_error.log}"
OUT="${2:-/tmp/astra_grasp_error.txt}"

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

timeout 300 ros2 launch launch/demo_gazebo_ur.launch.py use_rviz:=false \
  gz_args:="-s -r --physics-engine gz-physics-bullet-featherstone-plugin empty.sdf" \
  recipe:=recipes/ASTRA_Pranav_Variant_A.json > "$LOG" 2>&1 &

until ros2 topic list 2>/dev/null | grep -q '^/joint_states$'; do sleep 1; done
sleep 8   # let the scene spawn and the parts settle

for i in $(seq 1 24); do
  PART=$(timeout 5 gz model -m member_A -p 2>/dev/null \
         | grep -A1 'Pose' | tail -1 | tr -d '[]')
  TCP=$(timeout 5 ros2 run tf2_ros tf2_echo base_link grasp_tcp --once 2>/dev/null \
        | grep -A1 'Translation' | head -1)
  echo "t=$i part=[$PART] tcp=$TCP" >> "$OUT"
  sleep 2
done

kill_leftovers
echo "MEASURE_DONE" >> "$OUT"
