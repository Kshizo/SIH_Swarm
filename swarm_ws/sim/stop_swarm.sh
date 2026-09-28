#!/usr/bin/env bash
# Stop every process a run_swarm.sh run starts.
#
# Matching these reliably is fiddly: `gz sim` runs as a ruby process, so a name
# match never sees it, while an unanchored command-line match also matches the
# shell that happens to be carrying the pattern text. Anchoring the gz pattern
# to the start of the command line gets Gazebo and nothing else.
set -uo pipefail

names='arducopter|MicroXRCEAgent'
gz='^gz sim'
ros='parameter_bridge|async_slam_toolbox|person_detector|frontier_exploration|rf2o_laser_odometry|vio_to_ardupilot|hazard_mapper|ground_station|record_mission|robot_state_publisher'

count() {
  # `pgrep -c` exits non-zero on a zero count, so count lines instead.
  local named gz_count
  named=$(pgrep -x "$names" 2>/dev/null | wc -l)
  gz_count=$(pgrep -f "$gz" 2>/dev/null | wc -l)
  echo $(( named + gz_count ))
}

before=$(count)
for signal in TERM KILL; do
  pgrep -x "$names" 2>/dev/null | xargs -r kill "-$signal" 2>/dev/null
  pgrep -f "$gz"    2>/dev/null | xargs -r kill "-$signal" 2>/dev/null
  pgrep -f "$ros"   2>/dev/null | xargs -r kill "-$signal" 2>/dev/null
  sleep 2
done

after=$(count)
echo "stopped: $before simulation processes before, $after after"
[[ "$after" -eq 0 ]]
