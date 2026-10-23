#include <lcm/lcm-cpp.hpp>
#include <quad/imu_t.hpp>
#include <quad/joint_state_t.hpp>
#include <quad/motor_cmd_t.hpp>
#include <quad/rl_action_t.hpp>
#include <quad/robot_state_t.hpp>
#include <quad/safety_status_t.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

// TODO: add joint limits, comm timeout, motor fault handling, E-Stop

struct SafetyContext {
    lcm::LCM lcm;
    quad::imu_t latest_imu;
    quad::joint_state_t latest_joints;
    quad::motor_cmd_t latest_motor_cmd;
    quad::rl_action_t latest_action;
    quad::safety_status_t status;

    uint64_t last_motor_cmd_time = 0;
    uint64_t last_joint_state_time = 0;
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
