#!/usr/bin/env bash
# Bring up the SIH_Swarm simulation: Gazebo maze world, one ArduPilot SITL and
# one Micro-XRCE-DDS agent per drone, and (unless --no-stack) the ROS 2
# perception/exploration stack for each drone on its own ROS_DOMAIN_ID.
#
#   ./sim/run_swarm.sh                  # 2 drones, GUI, full stack
#   ./sim/run_swarm.sh --drones 1       # single drone
#   ./sim/run_swarm.sh --headless       # no Gazebo GUI
#   ./sim/run_swarm.sh --no-stack       # sim + autopilots only
#   ./sim/run_swarm.sh --record out.mp4 # also film the mission (see tools/record_mission.py)
#   ./sim/run_swarm.sh --gps            # fly on satellites instead of lidar odometry
#
# Everything is logged to sim/logs/<timestamp>/ and torn down on Ctrl-C.
set -uo pipefail

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SIM_DIR")"
ARDUPILOT="${ARDUPILOT_HOME:-$HOME/ardupilot}"
ARDUPILOT_GAZEBO="${ARDUPILOT_GAZEBO_HOME:-$HOME/ardupilot_gazebo}"
ARDU_WS="${ARDU_WS:-$HOME/ardu_ws}"
ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-/opt/ros/jazzy/setup.bash}"

DRONES=2
HEADLESS=0
RUN_STACK=1
RECORD=""
NAV_PARAMS="$WS_DIR/src/gps_denied.parm"
WORLD="$SIM_DIR/worlds/maze_survivors.sdf"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --drones)   DRONES="$2"; shift 2 ;;
    --headless) HEADLESS=1; shift ;;
    --no-stack) RUN_STACK=0; shift ;;
    --record)   RECORD="${2:-mission.mp4}"; shift 2 ;;
    --gps)      NAV_PARAMS="$SIM_DIR/config/gps_enabled.parm"; shift ;;
    --world)    WORLD="$2"; shift 2 ;;
    -h|--help)  sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

if [[ "$DRONES" != "1" && "$DRONES" != "2" ]]; then
  echo "--drones must be 1 or 2" >&2; exit 2
fi

log_dir="$SIM_DIR/logs/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$log_dir"
PIDS=()

die() { echo "ERROR: $*" >&2; exit 1; }

note() { printf '\033[1;36m==>\033[0m %s\n' "$*"; }

start() {  # start <name> <command...>
  local name="$1"; shift
  # setsid puts each component in its own process group so that cleanup can take
  # down the whole tree (ros2 launch in particular spawns a node per process).
  setsid "$@" >"$log_dir/$name.log" 2>&1 &
  local pid=$!
  PIDS+=("$pid")
  note "$name (pid $pid) -> $log_dir/$name.log"
}

cleanup() {
  echo
  note "shutting down"
  for pid in "${PIDS[@]}"; do
    kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
  done
  sleep 3
  for pid in "${PIDS[@]}"; do
    kill -KILL "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
  done
  wait 2>/dev/null
  note "logs in $log_dir"
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------- preflight --
[[ -x "$ARDUPILOT/build/sitl/bin/arducopter" ]] || die \
  "no SITL binary at $ARDUPILOT/build/sitl/bin/arducopter (build it with ./waf copter)"
[[ -f "$ARDUPILOT_GAZEBO/build/libArduPilotPlugin.so" ]] || die \
  "ardupilot_gazebo plugin not built at $ARDUPILOT_GAZEBO/build"
command -v MicroXRCEAgent >/dev/null || die "MicroXRCEAgent not on PATH"
command -v gz >/dev/null || die "gz (Gazebo Sim) not on PATH"
[[ -f "$WS_DIR/install/setup.bash" ]] || die \
  "workspace not built: run 'colcon build' in $WS_DIR first"

# A second simulation sharing this machine means two /clock sources, two SITLs
# on the same FDM ports and duplicate node names on the same domains. Nothing
# arms, and the logs blame TF. Refuse to start instead.
# Two matches, because one pattern cannot catch both: the autopilots and agents
# match on executable name, while `gz sim` runs as a ruby process and only shows
# up in the command line - anchored to its start, so the pattern does not also
# match the shell that happens to be carrying the pattern text.
# `pgrep -c` prints its count AND exits non-zero when that count is zero, so a
# `|| echo 0` fallback appends a second zero. Count lines instead.
stale_named=$(pgrep -x 'arducopter|MicroXRCEAgent' 2>/dev/null | wc -l)
stale_gz=$(pgrep -f '^gz sim' 2>/dev/null | wc -l)
stale=$(( stale_named + stale_gz ))
if [[ "$stale" -gt 0 ]]; then
  die "another simulation looks like it is still running ($stale processes).
  Stop it first:  ./sim/stop_swarm.sh"
fi

if [[ ! -f "$SIM_DIR/models/iris_with_sensors/model.sdf" ]]; then
  note "generating drone models"
  python3 "$SIM_DIR/tools/gen_drone_models.py" || die "model generation failed"
fi
[[ -f "$WORLD" ]] || { note "generating world"; python3 "$SIM_DIR/tools/gen_world.py"; }

# ------------------------------------------------------------------- gazebo --
export GZ_SIM_RESOURCE_PATH="$SIM_DIR/models:$ARDUPILOT_GAZEBO/models:$ARDUPILOT_GAZEBO/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

gz_args=(sim -v3 -r "$WORLD")
[[ "$HEADLESS" == "1" ]] && gz_args=(sim -v3 -r -s "$WORLD")
start gazebo gz "${gz_args[@]}"

note "waiting for Gazebo to advertise the lidar topic"
for _ in $(seq 1 60); do
  if gz topic -l 2>/dev/null | grep -q '/drone1/lidar/scan'; then break; fi
  sleep 1
done
gz topic -l 2>/dev/null | grep -q '/drone1/lidar/scan' \
  || die "Gazebo never advertised /drone1/lidar/scan -- see $log_dir/gazebo.log"

# ------------------------------------------------- autopilots + DDS agents --
# Instance i talks to the ArduPilotPlugin on FDM port 9002+10i, streams MAVLink
# to 14540+10*sysid (where vio_to_ardupilot is listening) and reaches ROS 2
# through its own XRCE agent.
for (( d=1; d<=DRONES; d++ )); do
  instance=$(( d - 1 ))
  mavlink_port=$(( 14540 + 10 * d ))
  agent_port=$(( 2018 + d ))
  yaw=$(( (d - 1) * 180 ))

  start "xrce_agent_drone$d" env ROS_DOMAIN_ID="$d" \
    MicroXRCEAgent udp4 -p "$agent_port"

  # Each autopilot needs its own working directory: SITL writes eeprom.bin,
  # logs/ and terrain/ into the cwd, and two instances sharing one would fight
  # over the same parameter storage.
  sitl_dir="$log_dir/sitl_drone$d.d"
  mkdir -p "$sitl_dir"
  start "sitl_drone$d" bash -c "
    cd '$sitl_dir'
    exec '$ARDUPILOT/build/sitl/bin/arducopter' \
      --model JSON \
      --serial0='udpclient:127.0.0.1:$mavlink_port' \
      --serial1='udpclient:127.0.0.1:$((mavlink_port + 1))' \
      --defaults '$ARDUPILOT/Tools/autotest/default_params/copter.parm,$ARDUPILOT/Tools/autotest/default_params/gazebo-iris.parm,$NAV_PARAMS,$SIM_DIR/config/drone$d.parm' \
      --home '-35.363262,149.165237,584,$yaw' \
      -I$instance"
done

# ------------------------------------------------------------------ ROS 2 --
if [[ "$RUN_STACK" == "1" ]]; then
  note "waiting for the autopilots to boot"
  sleep 15
  for (( d=1; d<=DRONES; d++ )); do
    start "stack_drone$d" env ROS_DOMAIN_ID="$d" bash -c "
      source '$ROS_DISTRO_SETUP'
      source '$ARDU_WS/install/setup.bash'
      source '$WS_DIR/install/setup.bash'
      exec ros2 launch drone_mapping beta_stack.launch.py"
  done
fi

# ---------------------------------------------------------------- recording --
if [[ -n "$RECORD" ]]; then
  case "$RECORD" in
    /*) record_out="$RECORD" ;;
    *)  record_out="$log_dir/$RECORD" ;;
  esac
  note "filming drone 1's mission"
  # The overhead camera lives in the world but nothing else bridges it.
  start observer_bridge env ROS_DOMAIN_ID=1 bash -c "
    source '$ROS_DISTRO_SETUP'
    exec ros2 run ros_gz_bridge parameter_bridge \
      /observer/image@sensor_msgs/msg/Image[gz.msgs.Image"
  start recorder bash -c "
    source '$ROS_DISTRO_SETUP'
    source '$ARDU_WS/install/setup.bash'
    source '$WS_DIR/install/setup.bash'
    exec python3 '$SIM_DIR/tools/record_mission.py' --out '$record_out' --drones $DRONES"
  note "  video -> $record_out"
fi

# ----------------------------------------------------------- ground station --
# The drones never talk to each other; their results are reconciled out here.
if [[ "$RUN_STACK" == "1" && "$DRONES" == "2" ]]; then
  start ground_station bash -c "
    source '$ROS_DISTRO_SETUP'
    source '$ARDU_WS/install/setup.bash'
    source '$WS_DIR/install/setup.bash'
    exec python3 '$WS_DIR/src/drone_control/drone_control/ground_station.py' \
      --drones 1,2 \
      --launch-poses '-9.6,-9.6,0 9.6,9.6,3.14159' \
      --expected 4 \
      --out '$log_dir/survivors.json'"
fi

note "navigation: $(basename "$NAV_PARAMS")"
note "running -- Ctrl-C to stop"
note "  RViz:      ROS_DOMAIN_ID=1 rviz2"
note "  MAVProxy:  mavproxy.py --master=udpin:127.0.0.1:14551   # drone 1 (drone 2: 14561)"
wait
