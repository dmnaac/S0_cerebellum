#include <lcm/lcm-cpp.hpp>
#include <quad/joint_state_t.hpp>
#include <quad/motor_cmd_t.hpp>

#include "core/robot_config.hpp"
#include "drivers/joint_motor.hpp"

#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

namespace {

volatile std::sig_atomic_t g_stop = 0;

void onSignal(int) {
    g_stop = 1;
}

} // namespace

struct MotorDriverContext {
    lcm::LCM lcm;
    core::RobotConfig config;
    std::vector<std::unique_ptr<cerebellum::drivers::JointMotor>> motors;
    quad::motor_cmd_t latest_cmd{};
    quad::joint_state_t joint_state{};
    bool cmd_ready = false;
};

void motorCmdHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                     const quad::motor_cmd_t* msg, MotorDriverContext* ctx) {
    ctx->latest_cmd = *msg;
    ctx->cmd_ready = true;
}

int main(int argc, char** argv) {
    std::string config_file = "config/motors.yaml";
    if (argc > 1) {
        config_file = argv[1];
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    MotorDriverContext ctx;
    if (!ctx.lcm.good()) {
        std::cerr << "motor_driver_node: LCM init failed" << std::endl;
        return -1;
    }

    if (!cerebellum::core::loadRobotConfig(config_file, ctx.config)) {
        std::cerr << "motor_driver_node: failed to load " << config_file << std::endl;
        return -1;
    }
    if (ctx.config.motors.empty()) {
        std::cerr << "motor_driver_node: no motors in config" << std::endl;
        return -1;
    }
    // loadRobotConfig sorts motors by CAN ID, so index i maps to motor ID i+1.
    const size_t num_motors = ctx.config.motors.size();

    ctx.motors.reserve(num_motors);
    for (const auto& mc : ctx.config.motors) {
        auto motor = cerebellum::drivers::createMotor(mc);
        if (!motor) {
            std::cerr << "motor_driver_node: unsupported protocol '" << mc.protocol
                      << "' for motor " << mc.name << std::endl;
            return -1;
        }
        if (!motor->init(mc)) {
            std::cerr << "motor_driver_node: failed to init motor " << mc.name << std::endl;
            return -1;
        }
        ctx.motors.push_back(std::move(motor));
    }

    std::cout << "motor_driver_node: enabling " << num_motors << " motors..." << std::endl;
    for (auto& m : ctx.motors) {
        m->enable();
    }

    ctx.lcm.subscribe("MOTOR_CMD", &motorCmdHandler, &ctx);
    std::cout << "motor_driver_node started" << std::endl;

    while (!g_stop) {
        auto start = std::chrono::steady_clock::now();

        ctx.lcm.handleTimeout(0);

        if (ctx.cmd_ready) {
            for (size_t i = 0; i < num_motors; ++i) {
                ctx.motors[i]->setCommand(
                    ctx.latest_cmd.position[i],
                    ctx.latest_cmd.velocity[i],
                    ctx.latest_cmd.kp[i],
                    ctx.latest_cmd.kd[i],
                    ctx.latest_cmd.torque[i]);

                cerebellum::drivers::JointState js{};
                ctx.motors[i]->readState(js);
                ctx.joint_state.position[i] = static_cast<float>(js.position);
                ctx.joint_state.velocity[i] = static_cast<float>(js.velocity);
                ctx.joint_state.torque[i] = static_cast<float>(js.torque);
                ctx.joint_state.temperature[i] = static_cast<float>(js.temperature);
                ctx.joint_state.error[i] = js.error;
            }

            ctx.joint_state.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch())
                                        .count();
            ctx.lcm.publish("JOINT_STATE", &ctx.joint_state);
        }

        std::this_thread::sleep_until(start + std::chrono::microseconds(1000)); // 1kHz
    }

    std::cout << "motor_driver_node: disabling motors..." << std::endl;
    for (auto& m : ctx.motors) {
        m->disable();
    }

    return 0;
}
