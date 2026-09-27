#!/bin/bash
# Run our solution inside the organizers' check-code container (their image, their checker, their bag).
# Mount this repo at /solution and the check-code folder at /workspace (as their scripts do), then:
#   bash /solution/tools/run_in_check_code.sh /workspace/bags/<bag_dir> [tram_id] [rate]
# Our tram_vehicle_msgs replaces the one shipped with check-code (it lacks DriverControllerCommand.msg).
set -e
BAG=${1:-/workspace/bags/30618_88aea4d9}; TRAM=${2:-30618}; RATE=${3:-1.0}; OUT=${OUT:-/tmp}
source /opt/ros/humble/setup.bash
WS=/tmp/jury_ws; mkdir -p $WS/src
cp -r /solution/ros2_ws/src/tram_odometry /solution/ros2_ws/src/tram_vehicle_msgs $WS/src/
[ -d /workspace/src/checker_ros ] && cp -r /workspace/src/checker_ros $WS/src/
cd $WS && colcon build --symlink-install > $OUT/build.log 2>&1 || { tail -30 $OUT/build.log; exit 1; }
echo "[check-code] built: tram_vehicle_msgs tram_odometry$( [ -d src/checker_ros ] && echo ' hackathon_solution_checker')"
source install/setup.bash
ros2 launch tram_odometry odometry.launch.py tram_id:=$TRAM > $OUT/node.log 2>&1 &
sleep 4
if [ -d src/checker_ros ]; then ros2 run hackathon_solution_checker metrics --ros-args -p report_period_sec:=30.0 > $OUT/checker.log 2>&1 & sleep 2; fi
echo "[check-code] playing $BAG at rate $RATE ..."
ros2 bag play "$BAG" --rate $RATE > $OUT/play.log 2>&1
sleep 2; pkill -INT -f hackathon_solution_checker 2>/dev/null || true; sleep 3; pkill -TERM -f odometry_node 2>/dev/null || true; pkill -TERM -f "ros2 launch" 2>/dev/null || true
[ -f $OUT/checker.log ] && { echo "[check-code] checker final report:"; grep -E "Velocity metrics|Position metrics" $OUT/checker.log | tail -2 | sed 's/\[INFO\] \[[0-9.]*\] \[hackathon_solution_checker\]: //'; }
echo "[check-code] node: $(grep -oE 'initialized from .*' $OUT/node.log | head -1)"; grep -E "rate .* Hz" $OUT/node.log | tail -1 | sed 's/.*tram_odometry\]: //'
