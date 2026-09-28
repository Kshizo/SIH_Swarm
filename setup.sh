#!/usr/bin/env bash
# One-time setup for a fresh checkout.
#
# Fetches the one dependency that is not vendored here, checks that the
# external tools are present, warns about the NumPy trap, and builds the
# workspace. Run it from the repository root:
#
#     ./setup.sh
#
# Then fly it with:  cd swarm_ws && ./sim/run_swarm.sh
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$REPO/swarm_ws"
ROS_SETUP="${ROS_DISTRO_SETUP:-/opt/ros/jazzy/setup.bash}"
ARDU_WS="${ARDU_WS:-$HOME/ardu_ws}"

ok()   { printf '  \033[32m*\033[0m %s\n' "$*"; }
warn() { printf '  \033[33m!\033[0m %s\n' "$*"; }
bad()  { printf '  \033[31mx\033[0m %s\n' "$*"; }
step() { printf '\n\033[1;36m==>\033[0m %s\n' "$*"; }

missing=0

# ------------------------------------------------------------- dependencies --
step "Checking prerequisites"

if [[ -f "$ROS_SETUP" ]]; then ok "ROS 2 at $ROS_SETUP"
else bad "ROS 2 not found at $ROS_SETUP (set ROS_DISTRO_SETUP)"; missing=1; fi

if command -v gz >/dev/null; then ok "Gazebo $(gz sim --version 2>/dev/null | head -1)"
else bad "gz (Gazebo Sim) not on PATH"; missing=1; fi

if [[ -x "${ARDUPILOT_HOME:-$HOME/ardupilot}/build/sitl/bin/arducopter" ]]; then
  ok "ArduPilot SITL built"
else
  bad "no SITL binary at ${ARDUPILOT_HOME:-$HOME/ardupilot}/build/sitl/bin/arducopter"
  warn "build it with: cd ~/ardupilot && ./waf configure --board sitl && ./waf copter"
  missing=1
fi

if [[ -f "${ARDUPILOT_GAZEBO_HOME:-$HOME/ardupilot_gazebo}/build/libArduPilotPlugin.so" ]]; then
  ok "ardupilot_gazebo plugin built"
else
  bad "ardupilot_gazebo not built (https://github.com/ArduPilot/ardupilot_gazebo)"
  missing=1
fi

if command -v MicroXRCEAgent >/dev/null; then ok "MicroXRCEAgent on PATH"
else bad "MicroXRCEAgent not on PATH"; missing=1; fi

if [[ -f "$ARDU_WS/install/setup.bash" ]]; then ok "ardupilot_msgs at $ARDU_WS"
else bad "ardupilot_msgs not found at $ARDU_WS (set ARDU_WS)"; missing=1; fi

# ----------------------------------------------------------- python packages --
step "Checking Python packages"

python3 - <<'PY'
import importlib, sys
need = {'cv2': 'python3-opencv', 'numpy': 'python3-numpy',
        'pymavlink': 'pip install pymavlink',
        'ai_edge_litert': 'pip install --user --break-system-packages ai_edge_litert'}
missing = []
for module, how in need.items():
    try:
        importlib.import_module(module)
        print('  \033[32m*\033[0m %s' % module)
    except ImportError:
        print('  \033[31mx\033[0m %s   -> %s' % (module, how))
        missing.append(module)

try:
    import numpy
    if int(numpy.__version__.split('.')[0]) >= 2:
        print('  \033[33m!\033[0m NumPy %s: the distro OpenCV and cv_bridge are built '
              'against 1.x and will fail to import.' % numpy.__version__)
        print('    Fix: pip uninstall -y --break-system-packages numpy')
except Exception:
    pass
sys.exit(1 if missing else 0)
PY
[[ $? -ne 0 ]] && missing=1

# ------------------------------------------------------------- the dependency --
step "Fetching rf2o_laser_odometry"
if [[ -f "$WS/src/rf2o_laser_odometry/package.xml" ]]; then
  ok "already present"
else
  rm -rf "$WS/src/rf2o_laser_odometry"
  if git clone -q -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git \
      "$WS/src/rf2o_laser_odometry"; then
    ok "cloned"
  else
    bad "clone failed - fetch it manually into swarm_ws/src/rf2o_laser_odometry"
    missing=1
  fi
fi

if [[ "$missing" -ne 0 ]]; then
  printf '\n\033[31mSetup incomplete.\033[0m Resolve the items above, then run ./setup.sh again.\n'
  exit 1
fi

# -------------------------------------------------------------------- build --
step "Building the workspace"
# shellcheck disable=SC1090
source "$ROS_SETUP"
source "$ARDU_WS/install/setup.bash"
cd "$WS" || exit 1
if colcon build --symlink-install; then
  ok "build succeeded"
else
  bad "build failed"
  exit 1
fi

step "Generating the simulation assets"
python3 "$WS/sim/tools/gen_drone_models.py" && python3 "$WS/sim/tools/gen_world.py"

cat <<EOF

$(printf '\033[1;32mReady.\033[0m')

  cd swarm_ws
  ./sim/run_swarm.sh                    two drones, Gazebo GUI, full stack
  ./sim/run_swarm.sh --headless         no GUI
  ./sim/run_swarm.sh --record run.mp4   film the mission

  ./sim/stop_swarm.sh                   stop everything

Watch it live:  ROS_DOMAIN_ID=1 rviz2   (/map, /scan, /person_map_pins, /hazard_markers)
EOF
