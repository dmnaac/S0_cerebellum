#!/bin/bash
# Launch all cerebellum nodes

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"

# Setup CAN
"$SCRIPT_DIR/setup_can.sh"

# Launch nodes
cd "$BUILD_DIR"

# Power board first: rail power must be up before joints are enabled
./power_board_node "$PROJECT_ROOT/config/power.yaml" &
PID_POWER=$!

./battery_node "$PROJECT_ROOT/config/battery.yaml" &
PID_BATTERY=$!

./command_node &
PID_CMD=$!

./sensor_node "$PROJECT_ROOT/config/robot.yaml" &
PID_SENSOR=$!

./rl_policy_node "$PROJECT_ROOT/config/states.yaml" "$PROJECT_ROOT/config/robot.yaml" &
PID_RL=$!

./motion_control_node &
PID_MC=$!

./motor_driver_node "$PROJECT_ROOT/config/motors.yaml" &
PID_MD=$!

./safety_monitor_node &
PID_SAFETY=$!

./logger_node &
PID_LOGGER=$!

echo "All nodes started. Press Ctrl+C to stop."

function cleanup {
    echo "Stopping nodes..."
    kill $PID_POWER $PID_BATTERY $PID_CMD $PID_SENSOR $PID_RL $PID_MC $PID_MD $PID_SAFETY $PID_LOGGER 2>/dev/null || true
    wait
}
trap cleanup SIGINT SIGTERM

wait
