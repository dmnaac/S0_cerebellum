#include <lcm/lcm-cpp.hpp>
#include <quad/battery_status_t.hpp>
#include <quad/imu_t.hpp>
#include <quad/joint_state_t.hpp>
#include <quad/motor_cmd_t.hpp>
#include <quad/power_status_t.hpp>
#include <quad/rl_action_t.hpp>
#include <quad/robot_state_t.hpp>
#include <quad/safety_status_t.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

// TODO: add joint limits, comm timeout, motor fault handling, E-Stop

// Power board voltage sanity bounds (V). TODO: tune to the actual battery
// pack and VMBUS ratings once known from bench measurements.
constexpr float POWER_VOLT_MIN = 18.0f;
constexpr float POWER_VOLT_MAX = 60.0f;
constexpr uint64_t POWER_STATUS_TIMEOUT_US = 500000;  // 500 ms

// BMS thresholds. TODO: tune to the actual pack once bench data is available.
constexpr uint64_t BATTERY_STATUS_TIMEOUT_US = 3000000;  // 3 s
constexpr int8_t BATTERY_RSOC_WARN = 20;
constexpr int8_t BATTERY_RSOC_CRITICAL = 10;
constexpr float BATTERY_CELL_DIFF_MAX_MV = 200.0f;

// Power board fault bits (see drivers/power_board_types.hpp)
constexpr int32_t POWER_FAULT_CRITICAL_MASK =
    (1 << 0) |  // 电源芯片过流
    (1 << 1) |  // 电源芯片过温
    (1 << 2);   // 电源板过温 (100 degC)

struct SafetyContext {
    lcm::LCM lcm;
    quad::imu_t latest_imu;
    quad::joint_state_t latest_joints;
    quad::motor_cmd_t latest_motor_cmd;
    quad::rl_action_t latest_action;
    quad::power_status_t latest_power;
    quad::battery_status_t latest_battery;
    quad::safety_status_t status;

    uint64_t last_motor_cmd_time = 0;
    uint64_t last_joint_state_time = 0;
    uint64_t last_power_status_time = 0;
    bool power_status_received = false;
    uint64_t last_battery_time = 0;
    bool battery_received = false;
};

void imuHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                const quad::imu_t* msg, SafetyContext* ctx) {
    ctx->latest_imu = *msg;
}

void jointHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                  const quad::joint_state_t* msg, SafetyContext* ctx) {
    ctx->latest_joints = *msg;
    ctx->last_joint_state_time = msg->utime;
}

void motorCmdHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                     const quad::motor_cmd_t* msg, SafetyContext* ctx) {
    ctx->latest_motor_cmd = *msg;
    ctx->last_motor_cmd_time = msg->utime;
}

void actionHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                   const quad::rl_action_t* msg, SafetyContext* ctx) {
    ctx->latest_action = *msg;
}

void powerStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                        const quad::power_status_t* msg, SafetyContext* ctx) {
    ctx->latest_power = *msg;
    ctx->last_power_status_time = msg->utime;
    ctx->power_status_received = true;
}

void batteryStatusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                          const quad::battery_status_t* msg, SafetyContext* ctx) {
    ctx->latest_battery = *msg;
    ctx->last_battery_time = msg->utime;
    ctx->battery_received = true;
}

void raiseLevel(SafetyContext& ctx, int level, const char* message) {
    if (level > ctx.status.level) {
        ctx.status.level = static_cast<int8_t>(level);
        ctx.status.message = message;
    }
    if (level >= 2) {
        ctx.status.e_stop = true;
    }
}

void checkSafety(SafetyContext& ctx) {
    auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count();

    ctx.status.e_stop = false;
    ctx.status.comm_timeout = false;
    ctx.status.level = 0;
    ctx.status.message = "OK";

    // Communication timeout check
    if (now_us - ctx.last_motor_cmd_time > 50000 || now_us - ctx.last_joint_state_time > 50000) {
        ctx.status.comm_timeout = true;
        ctx.status.level = 2;
        ctx.status.message = "Communication timeout";
        ctx.status.e_stop = true;
    }

    // Attitude check (placeholder quaternion to roll/pitch)
    // TODO: compute roll/pitch from quaternion
    // if (std::abs(roll) > 0.785 || std::abs(pitch) > 0.785) {
    //     ctx.status.level = 2;
    //     ctx.status.message = "Attitude out of range";
    //     ctx.status.e_stop = true;
    // }

    // Motor fault check
    for (int i = 0; i < 12; ++i) {
        if (ctx.latest_joints.error[i] != 0) {
            ctx.status.level = 2;
            ctx.status.message = "Motor fault detected";
            ctx.status.e_stop = true;
            break;
        }
    }

    // Power board checks
    if (ctx.power_status_received) {
        int32_t faults = ctx.latest_power.fault_bits & 0x7FFF;  // bit0~14

        if ((faults & POWER_FAULT_CRITICAL_MASK) != 0) {
            raiseLevel(ctx, 2, "Power board critical fault");
        } else if (faults != 0) {
            raiseLevel(ctx, 1, "Power board warning fault");
        }

        float battery_v = ctx.latest_power.battery_voltage;
        float motor_v = ctx.latest_power.motor_voltage;
        if ((battery_v > 0.0f && (battery_v < POWER_VOLT_MIN || battery_v > POWER_VOLT_MAX)) ||
            (motor_v > 0.0f && (motor_v < POWER_VOLT_MIN || motor_v > POWER_VOLT_MAX))) {
            raiseLevel(ctx, 1, "Power voltage out of range");
        }

        if (now_us - ctx.last_power_status_time > POWER_STATUS_TIMEOUT_US) {
            raiseLevel(ctx, 1, "Power status timeout");
        }
    }

    // BMS checks
    if (ctx.battery_received) {
        if (ctx.latest_battery.protection_bits != 0) {
            raiseLevel(ctx, 2, "BMS protection active");
        }
        if (ctx.latest_battery.error_code != 0) {
            raiseLevel(ctx, 1, "BMS read error");
        }
        if (ctx.latest_battery.rsoc_percent < BATTERY_RSOC_CRITICAL) {
            raiseLevel(ctx, 1, "Battery critically low, recharge soon");
        } else if (ctx.latest_battery.rsoc_percent < BATTERY_RSOC_WARN) {
            raiseLevel(ctx, 1, "Battery low");
        }

        float cell_min = ctx.latest_battery.cell_voltages[0];
        float cell_max = cell_min;
        for (int i = 1; i < 16; ++i) {
            float v = ctx.latest_battery.cell_voltages[i];
            if (v <= 0.0f) break;  // trailing zeros on <16S packs
            if (v < cell_min) cell_min = v;
            if (v > cell_max) cell_max = v;
        }
        if ((cell_max - cell_min) * 1000.0f > BATTERY_CELL_DIFF_MAX_MV) {
            raiseLevel(ctx, 1, "Battery cells unbalanced");
        }

        if (now_us - ctx.last_battery_time > BATTERY_STATUS_TIMEOUT_US) {
            raiseLevel(ctx, 1, "Battery status timeout");
        }
    }
}

int main(int argc, char** argv) {
    SafetyContext ctx;
    if (!ctx.lcm.good()) {
        std::cerr << "safety_monitor_node: LCM init failed" << std::endl;
        return -1;
    }

    ctx.lcm.subscribe("IMU", &imuHandler, &ctx);
    ctx.lcm.subscribe("JOINT_STATE", &jointHandler, &ctx);
    ctx.lcm.subscribe("MOTOR_CMD", &motorCmdHandler, &ctx);
    ctx.lcm.subscribe("RL_ACTION", &actionHandler, &ctx);
    ctx.lcm.subscribe("POWER_STATUS", &powerStatusHandler, &ctx);
    ctx.lcm.subscribe("BATTERY_STATUS", &batteryStatusHandler, &ctx);

    std::cout << "safety_monitor_node started" << std::endl;

    while (true) {
        auto start = std::chrono::steady_clock::now();

        ctx.lcm.handleTimeout(0);
        checkSafety(ctx);
        ctx.status.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                               std::chrono::steady_clock::now().time_since_epoch())
                               .count();
        ctx.lcm.publish("SAFETY_STATUS", &ctx.status);

        if (ctx.status.e_stop) {
            std::cerr << "E-STOP: " << ctx.status.message << std::endl;
            // TODO: trigger hardware/software emergency stop
        }

        std::this_thread::sleep_until(start + std::chrono::microseconds(1000)); // 1kHz
    }

    return 0;
}
