#include <lcm/lcm-cpp.hpp>
#include <quad/imu_t.hpp>
#include <quad/joint_state_t.hpp>
#include <quad/robot_state_t.hpp>
#include <quad/user_cmd_t.hpp>

#include <chrono>
#include <iostream>
#include <thread>

// TODO: integrate real IMU driver (USB/serial/SPI)

struct SensorContext {
    lcm::LCM lcm;
    quad::imu_t imu;
    quad::joint_state_t joints;
    quad::robot_state_t robot_state;
    quad::user_cmd_t latest_cmd;
};

void imuHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                const quad::imu_t* msg, SensorContext* ctx) {
    ctx->imu = *msg;
}

void jointHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                  const quad::joint_state_t* msg, SensorContext* ctx) {
    ctx->joints = *msg;
}

void cmdHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                const quad::user_cmd_t* msg, SensorContext* ctx) {
    ctx->latest_cmd = *msg;
}

void readImu(SensorContext& ctx) {
    // Placeholder: fill with dummy data
    ctx.imu.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count();
    ctx.imu.quaternion[0] = 1.0f;
    ctx.imu.quaternion[1] = 0.0f;
    ctx.imu.quaternion[2] = 0.0f;
    ctx.imu.quaternion[3] = 0.0f;
    for (int i = 0; i < 3; ++i) {
        ctx.imu.gyro[i] = 0.0f;
        ctx.imu.accel[i] = 0.0f;
    }
}

void aggregateRobotState(SensorContext& ctx) {
    ctx.robot_state.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::steady_clock::now().time_since_epoch())
                                .count();
    ctx.robot_state.imu = ctx.imu;
    ctx.robot_state.joints = ctx.joints;
    ctx.robot_state.cmd = ctx.latest_cmd;
    for (int i = 0; i < 12; ++i) {
        ctx.robot_state.previous_action[i] = 0.0f;
    }
}

int main(int argc, char** argv) {
    SensorContext ctx;
    if (!ctx.lcm.good()) {
        std::cerr << "sensor_node: LCM init failed" << std::endl;
        return -1;
    }

    ctx.lcm.subscribe("IMU", &imuHandler, &ctx);
    ctx.lcm.subscribe("JOINT_STATE", &jointHandler, &ctx);
    ctx.lcm.subscribe("USER_CMD", &cmdHandler, &ctx);

    std::cout << "sensor_node started" << std::endl;

    while (true) {
        auto start = std::chrono::steady_clock::now();

        ctx.lcm.handleTimeout(0);
        readImu(ctx);
        ctx.lcm.publish("IMU", &ctx.imu);

        aggregateRobotState(ctx);
        ctx.lcm.publish("ROBOT_STATE", &ctx.robot_state);

        std::this_thread::sleep_until(start + std::chrono::milliseconds(1));
    }

    return 0;
}
