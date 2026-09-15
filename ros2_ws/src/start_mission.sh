#!/usr/bin/env bash
# Start a field session in tmux: drivers in one pane, a topic monitor in another,
# and a shell ready to record.
#
#   start_mission            # drivers only
#   start_mission --record   # drivers + a rosbag recording into ~/data
#
# Detach with Ctrl-a d, reattach with `tmux attach -t mission`, stop with stop_mission.

set -euo pipefail

SESSION="mission"
WS="${HOME}/ros2_ws"
DATA_DIR="${HOME}/data"
RECORD=0

for arg in "$@"; do
  case "$arg" in
    --record) RECORD=1 ;;
    *) echo "Unknown argument: $arg" >&2; exit 1 ;;
  esac
done

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "Session '$SESSION' is already running. Attaching."
  exec tmux attach -t "$SESSION"
fi

SOURCE_CMD="source /opt/ros/\${ROS_DISTRO}/setup.bash && source ${WS}/install/setup.bash"

tmux new-session -d -s "$SESSION" -n drivers
tmux send-keys -t "$SESSION":drivers \
  "${SOURCE_CMD} && ros2 launch bluerov2_bringup bluerov2.launch.py" C-m

tmux new-window -t "$SESSION" -n monitor
tmux send-keys -t "$SESSION":monitor \
  "${SOURCE_CMD} && sleep 3 && watch -n 1 'ros2 topic hz /bluerov2/imu/data_raw --window 50 2>/dev/null | tail -3'" C-m

echo "Foxglove: connect to ws://$(hostname -I 2>/dev/null | awk '{print $1}'):8765"

tmux new-window -t "$SESSION" -n shell
tmux send-keys -t "$SESSION":shell "${SOURCE_CMD} && cd ${DATA_DIR}" C-m

if [[ "$RECORD" -eq 1 ]]; then
  mkdir -p "${DATA_DIR}"
  STAMP="$(date +%Y%m%d_%H%M%S)"
  tmux new-window -t "$SESSION" -n record
  # MCAP rather than sqlite3: it handles high-rate sensor data better and every tool
  # in the Foxglove ecosystem reads it natively.
  tmux send-keys -t "$SESSION":record \
    "${SOURCE_CMD} && cd ${DATA_DIR} && ros2 bag record -s mcap -o dive_${STAMP} \
       /bluerov2/imu/data /bluerov2/imu/data_raw /bluerov2/imu/mag \
       /bluerov2/dvl/velocity /bluerov2/dvl/report /bluerov2/dvl/altitude \
       /bluerov2/dvl/dead_reckoning \
       /bluerov2/camera/image_raw /bluerov2/camera/camera_info \
       /tf /tf_static" C-m
fi

echo "Mission session started. Attach with: tmux attach -t ${SESSION}"
tmux attach -t "$SESSION"
