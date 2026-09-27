#!/usr/bin/env bash
# LCM Solution — one-command launcher (Linux / macOS / Git Bash).
#   ./run.sh judge <path/to/bag_dir> [tram_id] [rate]   # node + bag play + record + metrics + resources
#   ./run.sh node  [tram_id]                            # only the node (host network) — play bags / judge from the host
#   ./run.sh shell                                      # interactive shell inside the image
# Uses the prebuilt image ghcr.io/tellsamm/lcm-odometry (pulled automatically); falls back to a local build.
set -e
IMAGE=${IMAGE:-ghcr.io/tellsamm/lcm-odometry:latest}
cd "$(dirname "$0")"
ensure_image() {
  # always try to fetch the latest published image (fast no-op if up to date); offline -> use local copy or build
  echo "[run] checking image $IMAGE ..."
  if docker pull -q "$IMAGE" >/dev/null 2>&1; then return; fi
  docker image inspect "$IMAGE" >/dev/null 2>&1 && { echo "[run] registry unreachable, using local image"; return; }
  echo "[run] pull failed -> building locally (3-5 min)"; docker build -t "$IMAGE" -f docker/Dockerfile .
}
case "${1:-}" in
  judge)
    if [ -z "${2:-}" ] || [ ! -d "$2" ]; then
      echo "ОШИБКА: папка прогона не найдена: '${2:-}'"
      echo "Укажите путь к папке ОДНОГО прогона на вашем диске (в ней лежат <имя>_0.db3 и metadata.yaml), например:"
      echo "Путь внутрь архива (.../data.zip/...) не подходит — сначала распакуйте архив."
      echo "  ./run.sh judge /home/user/hackathon/data/30618_0e41eac3"
      echo "  ./run.sh judge /c/hackathon/data/30618_0e41eac3        (Git Bash на Windows)"
      exit 1
    fi
    [ -f "$2/metadata.yaml" ] || { echo "ОШИБКА: в '$2' нет metadata.yaml — это не папка прогона. Нужна папка вида .../30618_0e41eac3"; exit 1; }
    BAG=$(cd "$2" && pwd -P)
    PARENT=$(dirname "$BAG"); NAME=$(basename "$BAG"); TRAM=${3:-30618}; RATE=${4:-1.0}
    if command -v cygpath >/dev/null 2>&1; then PARENT=$(cygpath -w "$PARENT"); OUT=$(cygpath -w "$PWD/results"); export MSYS_NO_PATHCONV=1; else OUT="$PWD/results"; fi
    mkdir -p results; ensure_image
    docker run --rm --cpus=2 --memory=512m -v "$PARENT:/data:ro" -v "$OUT:/out" "$IMAGE" bash /tools/judge_run.sh "/data/$NAME" "$TRAM" "$RATE" ;;
  node)
    TRAM=${2:-30618}; ensure_image
    docker run --rm -it --network host -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" "$IMAGE" ros2 launch tram_odometry odometry.launch.py tram_id:="$TRAM" ;;
  shell)
    ensure_image; docker run --rm -it --network host "$IMAGE" bash ;;
  *) sed -n '2,6p' "$0" ;;
esac
