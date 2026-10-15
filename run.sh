#!/usr/bin/env bash
# Drive the race track in CARLA: start the CARLA server if it is not running,
# then run the controllers (python -m carla_bridge). Run ./setup.sh first.
#
#   ./run.sh                                   NMPC, then pure pursuit, on CARLA's vehicle dynamics
#   ./run.sh --dynamics chrono                 on the Project Chrono sedan instead
#   ./run.sh --controller nmpc                 one controller (nmpc or pure_pursuit)
#   ./run.sh --stream 8080                     also serve the camera view live on port 8080
#   ./run.sh --perfect                         exact sensing (default: noise, 60 m range, road known 40 m ahead)
#   ./run.sh --laps 3 --no-video               (python -m carla_bridge -h lists every option)
#   ./run.sh --docker ...                      start CARLA from its Docker image, not external/carla
#   ./run.sh --stop-carla                      stop the CARLA server and exit
#   ./run.sh --offline [simulate options]      no CARLA at all: build/simulate (logs in results/offline)
#
# The CARLA server is left running between runs (it takes a minute to start).
# Results: results/carla/<controller>_<dynamics>.mp4, .csv, .json
set -euo pipefail
cd "$(dirname "$0")"
source scripts/env.sh

start_args=()
bridge_args=()
offline=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --docker) start_args+=(--docker) ;;
        --quality) start_args+=(--quality "$2"); shift ;;
        --stop-carla) exec scripts/start_carla.sh --stop ;;
        --offline) offline=1 ;;
        -h|--help) sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) bridge_args+=("$1") ;;
    esac
    shift
done

[[ -x $PY && -d build/python ]] || { echo "not set up yet: run ./setup.sh" >&2; exit 1; }
cmake --build build -j"$(nproc)" 2>&1 | grep -E "error" && exit 1  # rebuild after a change, if any

if [[ $offline == 1 ]]; then
    exec build/simulate "${bridge_args[@]}"
fi

scripts/start_carla.sh "${start_args[@]}"
exec "$PY" -m carla_bridge --port "$CARLA_PORT" "${bridge_args[@]}"
