#!/bin/bash
# Launch all cerebellum nodes

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"

# Setup CAN
"$SCRIPT_DIR/setup_can.sh"

# Start LCM multicast daemon if not running
if ! pgrep -x "lcm-logplayer-gui" > /dev/null; then
    echo "Starting LCM multicast daemon..."
    # lcm-spy or lcm-logger may start daemon automatically
fi

# Launch nodes
cd "$BUILD_DIR"

./command_node &
PID_CMD=$!

./sensor_node &
PID_SENSOR=$!

./rl_policy_node &
PID_RL=$!

./motion_control_node &
PID_MC=$!

./motor_driver_node &
PID_MD=$!

./safety_monitor_node &
PID_SAFETY=$!

./logger_node &
PID_LOGGER=$!

echo "All nodes started. Press Ctrl+C to stop."

function cleanup {
    echo "Stopping nodes..."
    kill $PID_CMD $PID_SENSOR $PID_RL $PID_MC $PID_MD $PID_SAFETY $PID_LOGGER 2>/dev/null || true
    wait
}
trap cleanup SIGINT SIGTERM

wait
