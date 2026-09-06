#!/bin/bash
# Single-instance guard. Two of these running at once is fatal in a way that
# looks like a code failure: the first to finish runs kill_leftovers and
# SIGKILLs the other's simulator mid-run, which shows up as every process
# dying with -9 and no explanation.
exec 9>/tmp/astra_sim.lock
if ! flock -n 9; then
  echo "another simulation run holds /tmp/astra_sim.lock; refusing to start" >&2
  exit 3
fi
RECIPE="${1:-recipes/ASTRA_Pranav_Variant_A.json}"
LOG="${2:-/tmp/astra_ur.log}"
PLANNER="${3:-moveit}"
kill_leftovers() {
  for pat in "[j]oint_state_restamp" "[r]obot_state_publisher" "[g]z sim" \
             "[r]un_job" "[p]arameter_bridge" "[s]tatic_transform_publisher" "[r]viz2" "[c]urobo_plan_server"; do
    pkill -9 -f "$pat" 2>/dev/null
  done
}
kill_leftovers; sleep 2
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/.."
timeout 300 ros2 launch launch/demo_gazebo_ur.launch.py use_rviz:=false \
  gz_args:="-s -r --physics-engine gz-physics-bullet-featherstone-plugin empty.sdf" \
  planner:="$PLANNER" \
  recipe:="$RECIPE" > "$LOG" 2>&1
kill_leftovers
echo "UR_SMOKE_DONE" >> "$LOG"
