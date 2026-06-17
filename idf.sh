#!/usr/bin/env bash
#
# Wrapper to run ESP-IDF v4.3.1 (the version this firmware is pinned to) inside
# the official espressif/idf docker image.
#
# This host is aarch64 but ESP-IDF 4.3 images are amd64-only, so we run the
# image under qemu emulation (--platform linux/amd64). Builds are slower than
# native but use the exact toolchain/APIs Bitcraze developed against.
#
# Usage:
#   ./idf.sh build                 # build the firmware
#   ./idf.sh fullclean             # wipe the build/ dir
#   ./idf.sh menuconfig            # interactive config (needs a tty)
#   ./idf.sh -p /dev/ttyUSB0 flash # flash (pass the serial device through)
#   ./idf.sh -p /dev/ttyUSB0 monitor
#
# Any arguments are passed straight through to idf.py.
set -euo pipefail

IMAGE="espressif/idf:release-v4.3"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Pass serial devices through to the container when flashing/monitoring.
DEVICE_ARGS=()
for dev in /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0; do
  [ -e "$dev" ] && DEVICE_ARGS+=(--device "$dev")
done

# Only allocate a tty when we actually have one. `-it` breaks non-interactive
# runs (e.g. `./idf.sh build` from a script/CI) with "cannot attach stdin to a
# TTY-enabled container". Interactive commands like menuconfig still get a tty.
TTY_ARGS=()
[ -t 0 ] && TTY_ARGS+=(-it)

exec docker run --rm \
  "${TTY_ARGS[@]}" \
  --platform linux/amd64 \
  -v "$PROJECT_DIR":/project \
  -w /project \
  "${DEVICE_ARGS[@]}" \
  "$IMAGE" \
  idf.py "$@"
