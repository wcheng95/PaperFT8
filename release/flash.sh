#!/usr/bin/env bash
# Bash helper to flash Mini-FT8 merged firmware image on Linux (ESP32-S3, 8MB)
# Usage:
#   1) Install esptool (recommended):
#        sudo apt install -y pipx
#        pipx ensurepath && exec $SHELL
#        pipx install esptool
#   2) chmod +x ./flash.sh
#   3) ./flash.sh

set -euo pipefail

BAUD=460800
CHIP="esp32s3"

require_nonempty() {
  local prompt="$1"
  local var
  while true; do
    read -r -p "$prompt" var
    var="${var#"${var%%[![:space:]]*}"}"   # ltrim
    var="${var%"${var##*[![:space:]]}"}"   # rtrim
    if [[ -n "$var" ]]; then
      echo "$var"
      return 0
    fi
    echo "Value required." >&2
  done
}

echo "Detected serial devices (quick scan):"
shopt -s nullglob
devs=(/dev/ttyACM* /dev/ttyUSB* /dev/cu.usbmodem* /dev/cu.usbserial*)
shopt -u nullglob
if ((${#devs[@]})); then
  for d in "${devs[@]}"; do
    echo "  $d"
  done
else
  echo "  (none found)"
fi
echo ""
echo "Tip: if permissions fail, run: sudo usermod -aG dialout \$USER  (then log out/in)"
echo ""

PORT="$(require_nonempty "Enter serial port (e.g., /dev/ttyACM0 or /dev/ttyUSB0): ")"
BIN_IN="$(require_nonempty "Enter merged firmware .bin path (e.g., MiniFT8_V1.3.2.bin): ")"

if [[ ! -f "$BIN_IN" ]]; then
  echo "ERROR: File not found: $BIN_IN" >&2
  exit 1
fi

echo ""
echo "Flashing $CHIP on $PORT @ $BAUD..."
echo "  File: $BIN_IN"

esptool --chip "$CHIP" -p "$PORT" -b "$BAUD" \
  --before default_reset --after hard_reset \
  write_flash \
  --flash-mode dio --flash-size 8MB --flash-freq 80m \
  0x0 "$BIN_IN"

echo "Done."
