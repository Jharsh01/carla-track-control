#!/usr/bin/env bash
# Start the CARLA 0.9.15 server off screen and wait until it answers.
#
#   scripts/start_carla.sh                 from external/carla (scripts/install_carla.sh)
#   scripts/start_carla.sh --docker        from the carlasim/carla:0.9.15 image instead (needs the NVIDIA
#                                          container toolkit; pulls ~9 GB the first time)
#   scripts/start_carla.sh --quality Low   rendering quality (Epic by default)
#   scripts/start_carla.sh --stop          stop the server this script started
#
# The port is CARLA_PORT (2000). Nothing happens if a server already answers
# there. The server's output goes to results/carla_server.log.
set -euo pipefail
cd "$(dirname "$0")/.."
source scripts/env.sh

DOCKER_NAME=carla-track-control
docker=0
quality=Epic
stop=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --docker) docker=1 ;;
        --quality) quality=$2; shift ;;
        --stop) stop=1 ;;
        -h|--help) sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done

pid_file=$EXTERNAL/carla.pid
answers() {
    "$PY" - "$CARLA_PORT" <<'EOF' >/dev/null 2>&1
import sys, carla
client = carla.Client("localhost", int(sys.argv[1]))
client.set_timeout(5.0)
client.get_server_version()
EOF
}

if [[ $stop == 1 ]]; then
    if docker ps -q -f name="^$DOCKER_NAME$" 2>/dev/null | grep -q .; then docker rm -f "$DOCKER_NAME" >/dev/null; fi
    if [[ -f $pid_file ]]; then
        pkill -TERM -P "$(cat "$pid_file")" 2>/dev/null || true
        kill "$(cat "$pid_file")" 2>/dev/null || true
        rm -f "$pid_file"
    fi
    pkill -f "CarlaUE4-Linux-Shipping.*carla-rpc-port=$CARLA_PORT" 2>/dev/null || true
    echo "   CARLA stopped"
    exit 0
fi

if answers; then
    echo "   CARLA already answering on port $CARLA_PORT"
    exit 0
fi

mkdir -p results "$EXTERNAL"
log=results/carla_server.log
args=(-RenderOffScreen -nosound -carla-rpc-port="$CARLA_PORT" -quality-level="$quality")
if [[ $docker == 1 ]]; then
    echo "   starting CARLA in Docker (carlasim/carla:0.9.15)"
    docker rm -f "$DOCKER_NAME" >/dev/null 2>&1 || true
    docker run -d --name "$DOCKER_NAME" --gpus all --net=host carlasim/carla:0.9.15 \
        /bin/bash ./CarlaUE4.sh "${args[@]}" >/dev/null
    (docker logs -f "$DOCKER_NAME" > "$log" 2>&1 &)
else
    [[ -x $CARLA_ROOT/CarlaUE4.sh ]] || { echo "no CARLA in $CARLA_ROOT: run scripts/install_carla.sh (or use --docker)" >&2; exit 1; }
    if [[ $EUID -eq 0 ]]; then
        echo "CARLA refuses to run as root; use an ordinary user (on Brev: a VM-mode instance, user ubuntu)" >&2
        exit 1
    fi
    echo "   starting CARLA from $CARLA_ROOT"
    nohup "$CARLA_ROOT/CarlaUE4.sh" "${args[@]}" > "$log" 2>&1 &
    echo $! > "$pid_file"
fi

echo -n "   waiting for the server"
for _ in $(seq 1 60); do
    if answers; then
        echo " - ready on port $CARLA_PORT"
        exit 0
    fi
    if [[ $docker == 0 ]] && ! kill -0 "$(cat "$pid_file")" 2>/dev/null; then
        echo; echo "CARLA exited during start-up; the end of $log:" >&2
        tail -20 "$log" >&2
        echo "(a Vulkan or GPU driver problem is the usual cause: vulkaninfo --summary, nvidia-smi)" >&2
        exit 1
    fi
    echo -n "."
    sleep 3
done
echo; echo "CARLA did not answer within 3 minutes; see $log" >&2
exit 1
