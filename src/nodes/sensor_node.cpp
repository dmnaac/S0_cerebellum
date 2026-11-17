#include <lcm/lcm-cpp.hpp>
#include <quad/imu_t.hpp>
#include <quad/joint_state_t.hpp>
#include <quad/robot_state_t.hpp>
#include <quad/user_cmd_t.hpp>

#include "core/robot_config.hpp"
#include "drivers/hipnuc_imu_driver.hpp"
#include "drivers/imu_driver.hpp"
#include "hipnuc_data.h"  // HIPNUC_STATUS_* bit definitions

#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

namespace {

volatile std::sig_atomic_t g_stop = 0;

void onSignal(int) {
    g_stop = 1;
}

constexpr int64_t ROBOT_STATE_PERIOD_US = 10000;  // 100 Hz aggregation

} // namespace

struct SensorContext {
    lcm::LCM lcm;
    cerebellum::core::RobotConfig config;
    std::unique_ptr<cerebellum::drivers::ImuDriver> imu;

    quad::imu_t imu_msg{};
    quad::joint_state_t joints{};
    quad::user_cmd_t latest_cmd{};
    quad::robot_state_t robot_state{};

    int64_t last_robot_state_time = 0;
    int64_t last_conv_warn_time = 0;
    bool imu_ok = false;
};

void jointHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                  const quad::joint_state_t* msg, SensorContext* ctx) {
    ctx->joints = *msg;
}

void cmdHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                const quad::user_cmd_t* msg, SensorContext* ctx) {
    ctx->latest_cmd = *msg;
}

void fillImuMessage(SensorContext& ctx, const cerebellum::drivers::ImuData& data) {
    quad::imu_t& msg = ctx.imu_msg;
    msg.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count();

    if (data.valid & cerebellum::drivers::IMU_VALID_QUAT) {
        for (int i = 0; i < 4; ++i) msg.quaternion[i] = data.quat[i];
    }
    if (data.valid & cerebellum::drivers::IMU_VALID_GYRO) {
        for (int i = 0; i < 3; ++i) msg.gyro[i] = data.gyro[i];
    }
    if (data.valid & cerebellum::drivers::IMU_VALID_ACCEL) {
        for (int i = 0; i < 3; ++i) msg.accel[i] = data.accel[i];
    }

    // Throttled convergence warning (ATT_CONV set = attitude not converged).
    if ((data.valid & cerebellum::drivers::IMU_VALID_STATUS) &&
        (data.main_status & HIPNUC_STATUS_ATT_CONV)) {
        if (msg.utime - ctx.last_conv_warn_time > 5000000) {
            std::cerr << "sensor_node: IMU attitude not yet converged" << std::endl;
            ctx.last_conv_warn_time = msg.utime;
        }
    }
}

void publishRobotState(SensorContext& ctx) {
    ctx.robot_state.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::steady_clock::now().time_since_epoch())
                                .count();
    ctx.robot_state.imu = ctx.imu_msg;
    ctx.robot_state.joints = ctx.joints;
    ctx.robot_state.cmd = ctx.latest_cmd;
    for (int i = 0; i < 12; ++i) {
        ctx.robot_state.previous_action[i] = 0.0f;
    }
    ctx.lcm.publish("ROBOT_STATE", &ctx.robot_state);
}

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::string config_file = "config/robot.yaml";
    if (argc > 1) {
        config_file = argv[1];
    }

    SensorContext ctx;
    if (!ctx.lcm.good()) {
        std::cerr << "sensor_node: LCM init failed" << std::endl;
        return -1;
    }

    if (!cerebellum::core::loadRobotConfig(config_file, ctx.config)) {
        std::cerr << "sensor_node: failed to load " << config_file << std::endl;
        return -1;
    }

    ctx.imu = cerebellum::drivers::createImuDriver(ctx.config.imu);
    if (!ctx.imu) {
        std::cerr << "sensor_node: unsupported IMU protocol '"
                  << ctx.config.imu.protocol << "'" << std::endl;
        return -1;
    }
    if (!ctx.imu->init(ctx.config.imu)) {
        return -1;
    }

    ctx.lcm.subscribe("JOINT_STATE", &jointHandler, &ctx);
    ctx.lcm.subscribe("USER_CMD", &cmdHandler, &ctx);

    std::cout << "sensor_node started" << std::endl;

    while (!g_stop) {
        ctx.lcm.handleTimeout(0);

        cerebellum::drivers::ImuData data;
        int ret = ctx.imu->read(data, ctx.config.imu.read_timeout_ms);
        if (ret == 1) {
            if (!ctx.imu_ok) {
                std::cout << "sensor_node: receiving IMU data" << std::endl;
                ctx.imu_ok = true;
            }
            fillImuMessage(ctx, data);
            ctx.lcm.publish("IMU", &ctx.imu_msg);
        } else if (ret < 0) {
            std::cerr << "sensor_node: IMU read error" << std::endl;
            break;
        }
        // ret == 0: timeout, no sample — device may be silent; keep looping

        auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                          std::chrono::steady_clock::now().time_since_epoch())
                          .count();
        if (now_us - ctx.last_robot_state_time >= ROBOT_STATE_PERIOD_US) {
            publishRobotState(ctx);
            ctx.last_robot_state_time = now_us;
        }
    }

    return 0;
}
