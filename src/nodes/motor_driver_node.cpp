#include <lcm/lcm-cpp.hpp>
#include <quad/joint_state_t.hpp>
#include <quad/motor_cmd_t.hpp>
#include <quad/safety_status_t.hpp>

#include <chrono>
#include <iostream>
#include <thread>

// TODO: integrate SocketCAN and concrete motor protocol

struct MotorContext {
    lcm::LCM lcm;
    quad::motor_cmd_t latest_cmd;
    quad::joint_state_t joint_state;
    quad::safety_status_t safety;
    bool cmd_ready = false;
};

void motorCmdHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                     const quad::motor_cmd_t* msg, MotorContext* ctx) {
    ctx->latest_cmd = *msg;
    ctx->cmd_ready = true;
}

void sendToMotors(const MotorContext& ctx) {
    // Placeholder: pack CAN frames and send via SocketCAN
    // for each motor:
    //   can_bus.send(frame);
}

void readFromMotors(MotorContext& ctx) {
    // Placeholder: read CAN feedback and fill joint_state
    ctx.joint_state.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::steady_clock::now().time_since_epoch())
                                .count();
    for (int i = 0; i < 12; ++i) {
        ctx.joint_state.position[i] = 0.0f;
        ctx.joint_state.velocity[i] = 0.0f;
        ctx.joint_state.torque[i] = 0.0f;
        ctx.joint_state.temperature[i] = 30.0f;
        ctx.joint_state.error[i] = 0;
    }
}

int main(int argc, char** argv) {
    MotorContext ctx;
    if (!ctx.lcm.good()) {
        std::cerr << "motor_driver_node: LCM init failed" << std::endl;
        return -1;
    }

    ctx.lcm.subscribe("MOTOR_CMD", &motorCmdHandler, &ctx);

    // TODO: initialize CAN buses based on config
    // std::vector<CanBus> buses;
    // for each motor config: buses.emplace_back(can_iface).open();

    std::cout << "motor_driver_node started" << std::endl;

    while (true) {
        auto start = std::chrono::steady_clock::now();

        ctx.lcm.handleTimeout(0);

        if (ctx.cmd_ready) {
            sendToMotors(ctx);
        }

        readFromMotors(ctx);
        ctx.lcm.publish("JOINT_STATE", &ctx.joint_state);

        std::this_thread::sleep_until(start + std::chrono::microseconds(1000)); // 1kHz
    }

    return 0;
}
