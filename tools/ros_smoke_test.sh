#!/bin/bash
# Live check inside the container: start the node, play a bag at given rate, sample outputs.
# usage: ros_smoke_test.sh <bag_dir> [rate] [tram_id]
set -e
BAG=$1; RATE=${2:-5}; TRAM=${3:-}
source /opt/ros/humble/setup.bash; source /ws/install/setup.bash
ros2 launch tram_odometry odometry.launch.py tram_id:=$TRAM > /tmp/node.log 2>&1 &
NODE=$!; sleep 3
ros2 bag play "$BAG" --rate $RATE --clock > /tmp/play.log 2>&1 &
PLAY=$!; sleep 6
echo "--- /result/velocity (1 msg)"; ros2 topic echo --once /result/velocity | head -8
echo "--- /result/position (1 msg)"; ros2 topic echo --once /result/position | head -16
echo "--- rates (5 s window)"; timeout 6 ros2 topic hz /result/position --window 100 2>&1 | grep -m1 "average rate" || true
timeout 6 ros2 topic hz /result/velocity --window 100 2>&1 | grep -m1 "average rate" || true
echo "--- diagnostics"; ros2 topic echo --once /result/diagnostics | grep -E "key|value" | head -12
wait $PLAY 2>/dev/null || true; sleep 1
echo "--- node log"; grep -E "initialized|rate|started" /tmp/node.log | tail -6
kill $NODE 2>/dev/null || true
