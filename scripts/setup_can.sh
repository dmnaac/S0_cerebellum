#!/bin/bash
# Setup SocketCAN interfaces for USB-CAN HUB

set -e

BITRATE=${BITRATE:-1000000}

for dev in can0 can1 can2 can3 can4; do
    if ip link show "$dev" &> /dev/null; then
        echo "Configuring $dev with bitrate $BITRATE"
        sudo ip link set "$dev" down || true
        sudo ip link set "$dev" type can bitrate "$BITRATE"
        sudo ip link set "$dev" up
    else
        echo "Interface $dev not found, skipping"
    fi
done

echo "CAN interfaces configured"
ip -s link show type can
