#!/usr/bin/env bash
# One-command hand-eye calibration session.
#
#   scripts/handeye_calibrate.sh <ROBOT_IP> [eye_to_hand|eye_in_hand] [extra launch args...]
#
# Checks the robot and camera are reachable, starts calibration.launch.py in the
# background (log: ~/.ros/handeye/session.log), then opens the interactive console.
# Quitting the console (q) shuts the whole session down.
#
# Examples
#   scripts/handeye_calibrate.sh 192.168.1.102
#   scripts/handeye_calibrate.sh 192.168.1.102 eye_in_hand
#   scripts/handeye_calibrate.sh 192.168.1.102 eye_to_hand use_motion:=true rviz:=false
#   scripts/handeye_calibrate.sh --console-only      # drivers + nodes already launched elsewhere
set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${HOME}/.ros/handeye"
LOG="${LOG_DIR}/session.log"

ok()   { printf '  \e[32mOK\e[0m  %s\n' "$*"; }
warn() { printf '  \e[33m!!\e[0m  %s\n' "$*"; }
die()  { printf '  \e[31mXX\e[0m  %s\n' "$*"; exit 1; }

usage() { sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }

# ---------------------------------------------------------------- environment
source_ros() {
  set +u   # ROS / colcon setup scripts read unset variables
  if [ -z "${ROS_DISTRO:-}" ]; then
    for d in humble jazzy; do
      if [ -f "/opt/ros/$d/setup.bash" ]; then
        # shellcheck disable=SC1090
        source "/opt/ros/$d/setup.bash"; break
      fi
    done
  fi
  [ -n "${ROS_DISTRO:-}" ] || die "ROS 2 not found. Install ROS 2 Humble or source it first."
  if [ ! -f "$WS/install/setup.bash" ]; then
    echo "Workspace not built yet; building..."
    (cd "$WS" && colcon build --symlink-install) || die "colcon build failed"
  fi
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  set -u
  ros2 pkg prefix ur5_bringup >/dev/null 2>&1 || die "ur5_bringup not found in $WS/install. Rebuild."
  ok "ROS 2 $ROS_DISTRO, workspace $WS"
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then usage 0; fi

if [ "${1:-}" = "--console-only" ]; then
  source_ros
  exec ros2 run handeye_calibration calibrate_cli
fi

[ $# -ge 1 ] || usage 1
ROBOT_IP="$1"; shift
MOUNT="eye_to_hand"
if [ $# -ge 1 ] && [[ "$1" != *:=* ]]; then MOUNT="$1"; shift; fi
case "$MOUNT" in
  eye_to_hand|eye_in_hand) ;;
  *) die "mount must be eye_to_hand or eye_in_hand, got '$MOUNT'" ;;
esac

echo "Preflight"
source_ros

# ---------------------------------------------------------------- robot
if ping -c 1 -W 2 "$ROBOT_IP" >/dev/null 2>&1; then
  ok "robot $ROBOT_IP answers ping"
else
  warn "robot $ROBOT_IP does not answer ping (check cable / IP / subnet)"
fi
if timeout 2 bash -c "exec 3<>/dev/tcp/$ROBOT_IP/29999" 2>/dev/null; then
  ok "UR dashboard server reachable (port 29999)"
else
  warn "UR dashboard port 29999 not reachable; the UR driver will not connect"
fi
HOST_IP="$(ip -4 route get "$ROBOT_IP" 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p')"
if [ -n "$HOST_IP" ]; then
  ok "this PC talks to the robot from $HOST_IP (External Control URCap host IP must be this)"
fi

# ---------------------------------------------------------------- camera
if command -v rs-enumerate-devices >/dev/null 2>&1; then
  if rs-enumerate-devices -s 2>/dev/null | grep -qi 'Intel RealSense'; then
    ok "RealSense: $(rs-enumerate-devices -s 2>/dev/null | grep -i 'Intel RealSense' | head -1 | awk '{$1=$1};1')"
  else
    warn "no RealSense camera found (USB 3 port? cable?)"
  fi
elif command -v lsusb >/dev/null 2>&1 && lsusb | grep -qi '8086:0b'; then
  ok "RealSense found on USB"
else
  warn "could not confirm a RealSense camera is plugged in"
fi

# ---------------------------------------------------------------- board config
CFG="$(ros2 pkg prefix ur5_bringup)/share/ur5_bringup/config/calibration.yaml"
SQ="$(sed -n 's/^ *square_length: *\([0-9.]*\).*/\1/p' "$CFG" | head -1)"
MK="$(sed -n 's/^ *marker_length: *\([0-9.]*\).*/\1/p' "$CFG" | head -1)"
DICT="$(sed -n 's/^ *dictionary: *\([A-Z0-9_]*\).*/\1/p' "$CFG" | head -1)"
ok "ChArUco board: square ${SQ} m, marker ${MK} m, ${DICT} (edit $CFG if your print differs)"

case "$MOUNT" in
  eye_to_hand) echo "  mount: eye_to_hand -> camera fixed in the cell, board bolted to the tool flange" ;;
  eye_in_hand) echo "  mount: eye_in_hand -> camera on the wrist, board fixed on the table" ;;
esac

# ---------------------------------------------------------------- launch
if ros2 node list 2>/dev/null | grep -qx '/handeye_calibration'; then
  die "a calibration session is already running. Stop it, or attach with: $0 --console-only"
fi

mkdir -p "$LOG_DIR"
echo
echo "Starting calibration.launch.py (log: $LOG)"
# Job control puts the launch in its own process group with SIGINT not ignored, so
# cleanup can stop every node it started, and Ctrl+C only reaches the console.
set -m
ros2 launch ur5_bringup calibration.launch.py robot_ip:="$ROBOT_IP" mount:="$MOUNT" "$@" \
  >"$LOG" 2>&1 &
LAUNCH_PID=$!
set +m

cleanup() {
  echo; echo "Shutting down the calibration session..."
  kill -INT -- "-$LAUNCH_PID" 2>/dev/null
  for _ in $(seq 30); do kill -0 "$LAUNCH_PID" 2>/dev/null || return 0; sleep 0.5; done
  kill -TERM -- "-$LAUNCH_PID" 2>/dev/null
}
trap cleanup EXIT

sleep 3
kill -0 "$LAUNCH_PID" 2>/dev/null || { tail -n 30 "$LOG"; die "launch exited early (see $LOG)"; }

cat <<EOF

Workflow
  1. Put the arm in freedrive (or jog it) so the board fills a good part of the image.
  2. Let it come to rest, press Enter here to take a sample.
  3. Repeat 15-20 times, tilting the wrist +/-30 deg or more about at least two axes.
  4. Type s to solve and save (~/.ros/handeye/ur5_handeye.yaml).
  Driver log: tail -f $LOG

EOF
ros2 run handeye_calibration calibrate_cli
