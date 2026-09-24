#!/usr/bin/env bash
# Checks the firmware toolchain and prints the environment the build needs.
set -uo pipefail

export PICO_SDK_PATH="${PICO_SDK_PATH:-/home/luis/pico/pico-sdk}"
export PICOTOOL_FETCH_FROM_GIT_PATH="${PICOTOOL_FETCH_FROM_GIT_PATH:-/home/luis/pico/picotool}"

missing=0
for tool in arm-none-eabi-gcc cmake ninja pkg-config; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "MISSING: $tool" >&2
        missing=1
    fi
done

if [ ! -d "$PICO_SDK_PATH" ]; then
    echo "MISSING: pico-sdk at $PICO_SDK_PATH" >&2
    echo "  git clone --depth 1 -b 2.3.0 https://github.com/raspberrypi/pico-sdk.git $PICO_SDK_PATH" >&2
    missing=1
else
    echo "pico-sdk: $(git -C "$PICO_SDK_PATH" describe --tags 2>/dev/null || echo unknown)"
fi

if [ "$missing" -ne 0 ]; then
    echo >&2
    echo "Install with:" >&2
    echo "  sudo apt install -y gcc-arm-none-eabi ninja-build libusb-1.0-0-dev pkg-config" >&2
    exit 1
fi

echo "toolchain OK"
echo "export PICO_SDK_PATH=$PICO_SDK_PATH"
echo "export PICOTOOL_FETCH_FROM_GIT_PATH=$PICOTOOL_FETCH_FROM_GIT_PATH"
