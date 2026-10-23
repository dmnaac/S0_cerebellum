#include <lcm/lcm-cpp.hpp>
#include <quad/joint_state_t.hpp>
#include <quad/motor_cmd_t.hpp>
#include <quad/rl_action_t.hpp>

#include <chrono>
#include <iostream>
#include <thread>

// TODO: add interpolation, PD gains, gravity compensation, impedance control

struct ControlContext {
    lcm::LCM lcm;
    quad::rl_action_t latest_action;
    quad::joint_state_t latest_joints;
    quad::motor_cmd_t motor_cmd;
    bool action_ready = false;
};

void actionHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                   const quad::rl_action_t* msg, ControlContext* ctx) {
    ctx->latest_action = *msg;
    ctx->action_ready = true;
}

void jointHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                  const quad::joint_state_t* msg, ControlContext* ctx) {
    ctx->latest_joints = *msg;
}

void computeMotorCommand(ControlContext& ctx) {
    for (int i = 0; i < 12; ++i) {
        // position_offset now carries absolute target joint position
        ctx.motor_cmd.position[i] = ctx.latest_action.position_offset[i];
        ctx.motor_cmd.velocity[i] = 0.0f;
        ctx.motor_cmd.kp[i] = 50.0f;
        ctx.motor_cmd.kd[i] = 2.0f;
        ctx.motor_cmd.torque[i] = 0.0f;
    }
}

int main(int argc, char** argv) {
    ControlContext ctx;
    if (!ctx.lcm.good()) {
        std::cerr << "motion_control_node: LCM init failed" << std::endl;
        return -1;
    }

    ctx.lcm.subscribe("RL_ACTION", &actionHandler, &ctx);
    ctx.lcm.subscribe("JOINT_STATE", &jointHandler, &ctx);

    std::cout << "motion_control_node started" << std::endl;

    while (true) {
        auto start = std::chrono::steady_clock::now();

        ctx.lcm.handleTimeout(0);

        if (ctx.action_ready) {
            computeMotorCommand(ctx);
            ctx.motor_cmd.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count();
            ctx.lcm.publish("MOTOR_CMD", &ctx.motor_cmd);
        }

        std::this_thread::sleep_until(start + std::chrono::microseconds(1000)); // 1kHz
    }

    return 0;
}
