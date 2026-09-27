#!/bin/bash
# Runs INSIDE our container. Builds the organizers' checker (mounted at /check) and evaluates our node with it,
# exactly as the jury does: reference = /localization/kinematic_state from the bag.
# usage: organizers_check.sh /data/<bag_dir> [tram_id] [rate]
set -e
BAG=$1; TRAM=${2:-30618}; RATE=${3:-1.0}
source /opt/ros/humble/setup.bash; source /ws/install/setup.bash
if [ ! -f /tmp/checker_ws/install/setup.bash ]; then
  mkdir -p /tmp/checker_ws/src && cp -r /check/src/checker_ros /tmp/checker_ws/src/
  (cd /tmp/checker_ws && colcon build --packages-select hackathon_solution_checker > /tmp/checker_build.log 2>&1) || { tail -20 /tmp/checker_build.log; exit 1; }
fi
source /tmp/checker_ws/install/setup.bash
ros2 launch tram_odometry odometry.launch.py tram_id:=$TRAM > /out/node.log 2>&1 &
NODE=$!; sleep 3
ros2 run hackathon_solution_checker metrics --ros-args -p report_period_sec:=30.0 > /out/checker.log 2>&1 &
CHK=$!; sleep 2
echo "[check] playing $BAG at rate $RATE ..."
ros2 bag play "$BAG" --rate $RATE > /out/play.log 2>&1
sleep 2
kill -INT $CHK 2>/dev/null; sleep 3; pkill -INT -f hackathon_solution_checker 2>/dev/null || true; sleep 1
kill -INT $NODE 2>/dev/null; sleep 2; pkill -TERM -f odometry_node 2>/dev/null || true; pkill -TERM -f "ros2 launch" 2>/dev/null || true
echo "[check] organizers' checker, final report:"; grep -E "Velocity metrics|Position metrics" /out/checker.log | tail -2
echo "[check] our node:"; grep -E "initialized|rate" /out/node.log | tail -2
