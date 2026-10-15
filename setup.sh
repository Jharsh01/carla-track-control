#!/usr/bin/env bash
# Set up everything on a fresh GPU machine: written for an NVIDIA Brev instance
# in VM mode (Ubuntu 22.04, NVIDIA driver, sudo); any Ubuntu 22.04/24.04 with an
# NVIDIA GPU should do. After it: ./run.sh
#
#   ./setup.sh                 everything (about an hour, most of it the CARLA download and the Chrono build)
#   ./setup.sh --no-carla      without the CARLA server: the offline simulation and the bridge's tests only
#   ./setup.sh --no-chrono     without Project Chrono: CARLA dynamics only
#   ./setup.sh --no-system     without apt (needs no sudo; install the packages listed below yourself)
#   ./setup.sh --jobs 8        parallel build jobs (default: all cores)
#
# A step that is already done is skipped, so run it again after a failure or an
# interrupted download.
#
# What goes where:
#   apt packages            compiler, CMake, Eigen, what the CARLA server needs (Vulkan, SDL)
#   ~/.local/bin/uv         makes the Python environment, downloading Python 3.8 itself
#   .venv/                  Python 3.8: CARLA client 0.9.15, CasADi (C++ library and IPOPT included), pybind11, ...
#   external/chrono/        Project Chrono 9.0.1, built here (scripts/install_chrono.sh)
#   external/carla/         CARLA 0.9.15 (scripts/install_carla.sh)
#   build/                  the library, the offline simulation (build/simulate), the Python module trackctl
set -euo pipefail
cd "$(dirname "$0")"

system=1 carla=1 chrono=1 jobs=$(nproc)
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-system) system=0 ;;
        --no-carla) carla=0 ;;
        --no-chrono) chrono=0 ;;
        --jobs) jobs=$2; shift ;;
        -h|--help) sed -n '2,24p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $1 (see --help)" >&2; exit 2 ;;
    esac
    shift
done

source scripts/env.sh
step() { echo; echo "== $*"; }
die() { echo "error: $*" >&2; exit 1; }

step "machine"
( . /etc/os-release 2>/dev/null && echo "   $PRETTY_NAME" ) || true
if command -v nvidia-smi >/dev/null; then
    nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader | sed 's/^/   GPU: /'
else
    echo "   warning: nvidia-smi not found; the CARLA server needs an NVIDIA GPU and its driver"
fi
echo "   $(nproc) cores, $(free -g | awk '/Mem:/ {print $2}') GB memory, $(df -Ph . | awk 'NR==2 {print $4}') free here"
[[ $EUID -eq 0 ]] && echo "   warning: running as root; the CARLA server refuses to start as root"

if [[ $system == 1 ]]; then
    step "system packages"
    sudo=(); [[ $EUID -eq 0 ]] || sudo=(sudo)
    "${sudo[@]}" apt-get update -q
    # build tools and Eigen; what the CARLA server loads (as in its Docker image) and Vulkan diagnostics; a font for the HUD
    "${sudo[@]}" env DEBIAN_FRONTEND=noninteractive apt-get install -y -q --no-install-recommends \
        build-essential cmake git curl ca-certificates libeigen3-dev \
        libomp5 libvulkan1 vulkan-tools xdg-user-dirs libsdl2-2.0-0 libgl1 libglib2.0-0 \
        fonts-dejavu-core
fi
for tool in git curl cmake c++; do command -v $tool >/dev/null || die "$tool not found"; done

if ! command -v uv >/dev/null; then
    step "uv"
    curl -LsSf https://astral.sh/uv/install.sh | sh
    command -v uv >/dev/null || die "uv was installed but is not on PATH"
fi
cmake_version=$(cmake --version | awk 'NR==1 {print $3}')
if [[ $(printf '%s\n3.18\n' "$cmake_version" | sort -V | head -1) != 3.18 ]]; then
    step "CMake $cmake_version is older than 3.18: installing a newer one with uv"
    uv tool install cmake
    hash -r
fi

step "Python environment (.venv, Python 3.8)"
[[ -x $PY ]] || uv venv -q --python 3.8 .venv
uv pip install -q --python "$PY" -r requirements.txt
"$PY" -c "import carla, casadi, pybind11, numpy, PIL, imageio_ffmpeg; print('   CARLA client 0.9.15, CasADi', casadi.__version__)"

if [[ $chrono == 1 ]]; then
    step "Project Chrono ($CHRONO_ROOT)"
    scripts/install_chrono.sh --jobs "$jobs"
fi

step "build (build/)"
mkdir -p build
cmake -S . -B build -DPython3_EXECUTABLE="$PY" > build/cmake.log 2>&1 || { tail -20 build/cmake.log; exit 1; }
grep -E "Project Chrono" build/cmake.log | sed 's/^-- /   /' || true
cmake --build build -j"$jobs" 2>&1 | grep -E "error|warning: unused" || true
[[ -x build/simulate ]] || die "the build failed; run: cmake --build build"

step "tests"
(cd build && ctest --output-on-failure --no-tests=error -j"$jobs") | tail -4
"$PY" -m pytest -q -m "not slow" | tail -2

if [[ $carla == 1 ]]; then
    step "CARLA 0.9.15 ($CARLA_ROOT)"
    scripts/install_carla.sh
fi

cat <<EOF

== set up
   ./run.sh                                  both controllers round the track in CARLA, on CARLA's dynamics
   ./run.sh --dynamics chrono                the same on the Project Chrono sedan
   ./run.sh --controller nmpc --stream 8080  one controller, watched live (README: "Watching live")
   ./run.sh --offline                        no CARLA: the offline simulation (build/simulate)
   Videos and logs go to results/carla/.
EOF
