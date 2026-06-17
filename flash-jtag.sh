#!/usr/bin/env bash
#
# Flash the AI-deck ESP32 (NINA) over JTAG using the Olimex ARM-USB-TINY-H.
#
# Why JTAG and not `idf.py flash`?
#   The AI-deck has no UART auto-reset path to the ESP32, so esptool's serial
#   download mode ("idf.py flash") just times out with "No serial data
#   received". The Olimex ARM-USB-TINY-H is a JTAG programmer; OpenOCD drives
#   it over libusb. That's the method the firmware README documents.
#
# Requirements:
#   - Olimex ARM-USB-TINY-H connected to the deck's ESP debug port (powered).
#   - `./idf.sh build` already run so build/ contains the binaries.
#
# Usage:
#   ./flash-jtag.sh            # flash bootloader + partition table + app, then reset
#
set -euo pipefail

IMAGE="espressif/idf:release-v4.3"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for f in build/bootloader/bootloader.bin \
         build/partition_table/partition-table.bin \
         build/aideck_esp.bin; do
  if [ ! -f "$PROJECT_DIR/$f" ]; then
    echo "Missing $f -- run ./idf.sh build first." >&2
    exit 1
  fi
done

# OpenOCD reaches the FT2232 (JTAG) over libusb, so pass the USB bus through
# (--privileged + /dev/bus/usb), NOT --device /dev/ttyUSB0 (that's the serial
# channel and is not wired to the ESP32 bootstrap pins).
exec docker run --rm -it \
  --platform linux/amd64 \
  --privileged -v /dev/bus/usb:/dev/bus/usb \
  -v "$PROJECT_DIR":/project -w /project \
  "$IMAGE" \
  openocd -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f board/esp-wroom-32.cfg \
    -c 'adapter_khz 20000' \
    -c 'program_esp build/bootloader/bootloader.bin 0x1000 verify' \
    -c 'program_esp build/partition_table/partition-table.bin 0x8000 verify' \
    -c 'program_esp build/aideck_esp.bin 0x10000 verify reset exit'
