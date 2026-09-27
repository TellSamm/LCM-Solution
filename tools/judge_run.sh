#!/bin/bash
# Runs INSIDE the container. Starts the node, replays a bag in real time, records /result/* topics,
# then computes accuracy metrics against the GNSS reference contained in the same bag.
# usage: judge_run.sh /data/<bag_dir> [tram_id] [rate]
set -e
BAG=$1; TRAM=${2:-}; RATE=${3:-1.0}
NAME=$(basename "$BAG"); OUT=/out/${NAME}_result
source /opt/ros/humble/setup.bash; source /ws/install/setup.bash
mkdir -p /out; rm -rf "$OUT"
echo "[judge] запуск ноды tram_odometry (калибровка трамвая: ${TRAM:-усреднённая})"
ros2 launch tram_odometry odometry.launch.py tram_id:=$TRAM > /out/${NAME}_node.log 2>&1 &
NODE=$!; sleep 3
echo "[judge] запись /result/velocity /result/position /result/diagnostics -> $OUT"
ros2 bag record -o "$OUT" /result/velocity /result/position /result/diagnostics > /out/${NAME}_record.log 2>&1 &
REC=$!; sleep 2
# resource sampling of the node process (cpu %, rss MB) every 5 s
PID=$(pgrep -f odometry_node | head -1)
( while kill -0 $PID 2>/dev/null; do ps -o %cpu=,rss= -p $PID | awk -v t="$(date +%s)" '{printf "%s cpu=%s%% rss=%.1fMB\n", t, $1, $2/1024}'; sleep 5; done ) > /out/${NAME}_resources.log &
echo "[judge] воспроизведение $BAG, скорость x$RATE (1.0 = реальное время). Ждите окончания bag ..."
ros2 bag play "$BAG" --rate $RATE > /out/${NAME}_play.log 2>&1
sleep 2
kill -INT $REC 2>/dev/null; sleep 2; pkill -INT -f "ros2 bag record" 2>/dev/null || true
kill -INT $NODE 2>/dev/null; sleep 2; pkill -TERM -f odometry_node 2>/dev/null || true; pkill -TERM -f "ros2 launch" 2>/dev/null || true
echo "[judge] инициализация: $(grep -oE "initialized from .*" /out/${NAME}_node.log | head -1 || echo "см. лог")"
echo "[judge] ресурсы: $(awk '{c+=substr($2,5); m=substr($3,5)+0>m?substr($3,5)+0:m; n++} END {printf "avg cpu %.1f%%, max rss %.1f MB (%d samples)", c/n, m, n}' /out/${NAME}_resources.log)"
echo "[judge] расчёт метрик относительно эталона из bag ..."
python3 /tools/eval_recorded.py "$BAG" "$OUT" /out/${NAME}_node.log /out/${NAME}_resources.log 2>&1 | grep -v "^\[INFO\]"
