#!/usr/bin/env bash
# Build Project Chrono 9.0.1 (core, vehicle, vehicle models) into external/chrono/install.
#
#   scripts/install_chrono.sh            build and install (skipped if already installed)
#   scripts/install_chrono.sh --jobs 8   parallel jobs (default: all cores)
#
# Chrono is compiled with the old libstdc++ string ABI, like everything that
# links with CasADi's pip wheel (CMakeLists.txt), and with -march=native (its
# default), so build it on the machine that runs it. About 20-40 minutes on 8
# cores; the source and build folders are removed afterwards (~1 GB installed).
set -euo pipefail
cd "$(dirname "$0")/.."
source scripts/env.sh

CHRONO_URL=https://github.com/projectchrono/chrono.git
CHRONO_TAG=9.0.1
jobs=$(nproc)
while [[ $# -gt 0 ]]; do
    case "$1" in
        --jobs) jobs=$2; shift ;;
        -h|--help) sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done

install="$CHRONO_ROOT/install"
if [[ -f $install/lib/cmake/Chrono/ChronoConfig.cmake && -f $install/.chrono-$CHRONO_TAG ]]; then
    echo "   Chrono $CHRONO_TAG already installed in $install"
    exit 0
fi

src="$CHRONO_ROOT/src" build="$CHRONO_ROOT/build"
if [[ ! -d $src/.git ]]; then
    echo "   cloning Chrono $CHRONO_TAG"
    git -c advice.detachedHead=false clone -q --depth 1 --branch "$CHRONO_TAG" "$CHRONO_URL" "$src"
fi
echo "   configuring"
cmake -S "$src" -B "$build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$install" \
    -DCMAKE_CXX_FLAGS="-D_GLIBCXX_USE_CXX11_ABI=0" \
    -DBUILD_DEMOS=OFF -DBUILD_TESTING=OFF -DBUILD_BENCHMARKING=OFF \
    -DENABLE_MODULE_VEHICLE=ON -DENABLE_MODULE_VEHICLE_MODELS=ON -DENABLE_MODULE_VEHICLE_COSIM=OFF \
    > "$CHRONO_ROOT/cmake.log" 2>&1 || { tail -30 "$CHRONO_ROOT/cmake.log"; exit 1; }
echo "   building with $jobs jobs (log: $CHRONO_ROOT/build.log)"
cmake --build "$build" -j"$jobs" > "$CHRONO_ROOT/build.log" 2>&1 || { tail -30 "$CHRONO_ROOT/build.log"; exit 1; }
cmake --install "$build" > "$CHRONO_ROOT/install.log" 2>&1
touch "$install/.chrono-$CHRONO_TAG"
rm -rf "$src" "$build"
echo "   installed in $install"
