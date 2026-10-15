#!/usr/bin/env bash
# Download CARLA 0.9.15 (the packaged Linux release, 8.4 GB) into external/carla.
#
# The race track is generated from OpenDRIVE at run time, so CARLA's additional
# maps are not needed. A partial download is continued when the script is run
# again. CARLA_URL overrides the address.
set -euo pipefail
cd "$(dirname "$0")/.."
source scripts/env.sh

# the Backblaze mirror listed by CARLA and Bench2Drive; tiny.carla.org links answered 403 when this was written
CARLA_URL=${CARLA_URL:-https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_0.9.15.tar.gz}
DISK_NEEDED_GB=25  # the archive and the unpacked server, until the archive is removed

if [[ -f $CARLA_ROOT/.carla-0.9.15 ]]; then
    echo "   CARLA 0.9.15 already installed in $CARLA_ROOT"
    exit 0
fi
mkdir -p "$CARLA_ROOT" "$EXTERNAL/downloads"
free_gb=$(df -Pk "$EXTERNAL" | awk 'NR==2 {print int($4 / 1048576)}')
if (( free_gb < DISK_NEEDED_GB )); then
    echo "   warning: $free_gb GB free in $EXTERNAL; about $DISK_NEEDED_GB GB are needed"
fi

archive=$EXTERNAL/downloads/CARLA_0.9.15.tar.gz
if [[ ! -f $archive ]]; then
    echo "   downloading CARLA 0.9.15 (8.4 GB)"
    curl -fL --retry 5 --retry-delay 10 -C - -o "$archive.part" "$CARLA_URL" \
        || { echo "download failed: $CARLA_URL (run the script again to continue it)" >&2; exit 1; }
    mv "$archive.part" "$archive"
fi
echo "   unpacking"
tar -xzf "$archive" -C "$CARLA_ROOT"
[[ -f $CARLA_ROOT/CarlaUE4.sh ]] || { echo "CarlaUE4.sh is not in the archive from $CARLA_URL" >&2; exit 1; }
touch "$CARLA_ROOT/.carla-0.9.15"
rm -f "$archive"
echo "   installed in $CARLA_ROOT"
