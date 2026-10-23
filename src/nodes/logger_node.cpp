#include <lcm/lcm-cpp.hpp>
#include <quad/imu_t.hpp>
#include <quad/joint_state_t.hpp>
#include <quad/motor_cmd_t.hpp>
#include <quad/rl_action_t.hpp>
#include <quad/robot_state_t.hpp>
#include <quad/safety_status_t.hpp>
#include <quad/user_cmd_t.hpp>

#include <fstream>
#include <iostream>
#include <mutex>
#include <string>

// TODO: add structured binary log format (e.g., MCAP or custom)

struct LoggerContext {
    lcm::LCM lcm;
    std::ofstream file;
    std::mutex mutex;
};

void logMessage(LoggerContext& ctx, const std::string& channel, int64_t utime) {
    std::lock_guard<std::mutex> lock(ctx.mutex);
    if (ctx.file.is_open()) {
        ctx.file << utime << " " << channel << std::endl;
    }
}

void userCmdHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                    const quad::user_cmd_t* msg, LoggerContext* ctx) {
    logMessage(*ctx, channel, msg->utime);
}

void imuHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                const quad::imu_t* msg, LoggerContext* ctx) {
    logMessage(*ctx, channel, msg->utime);
}

void jointHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                  const quad::joint_state_t* msg, LoggerContext* ctx) {
    logMessage(*ctx, channel, msg->utime);
}

void robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                       const quad::robot_state_t* msg, LoggerContext* ctx) {
    logMessage(*ctx, channel, msg->utime);
}

void actionHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                   const quad::rl_action_t* msg, LoggerContext* ctx) {
    logMessage(*ctx, channel, msg->utime);
}

void motorCmdHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                     const quad::motor_cmd_t* msg, LoggerContext* ctx) {
    logMessage(*ctx, channel, msg->utime);
}

void safetyHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                   const quad::safety_status_t* msg, LoggerContext* ctx) {
    logMessage(*ctx, channel, msg->utime);
}

int main(int argc, char** argv) {
    LoggerContext ctx;
    if (!ctx.lcm.good()) {
        std::cerr << "logger_node: LCM init failed" << std::endl;
        return -1;
    }

    std::string log_file = "log/cerebellum.log";
    if (argc > 1) {
        log_file = argv[1];
    }

    ctx.file.open(log_file, std::ios::out | std::ios::app);
    if (!ctx.file.is_open()) {
        std::cerr << "logger_node: failed to open " << log_file << std::endl;
        return -1;
    }

    ctx.lcm.subscribe("USER_CMD", &userCmdHandler, &ctx);
    ctx.lcm.subscribe("IMU", &imuHandler, &ctx);
    ctx.lcm.subscribe("JOINT_STATE", &jointHandler, &ctx);
    ctx.lcm.subscribe("ROBOT_STATE", &robotStateHandler, &ctx);
    ctx.lcm.subscribe("RL_ACTION", &actionHandler, &ctx);
    ctx.lcm.subscribe("MOTOR_CMD", &motorCmdHandler, &ctx);
    ctx.lcm.subscribe("SAFETY_STATUS", &safetyHandler, &ctx);

    std::cout << "logger_node started, logging to " << log_file << std::endl;

    while (true) {
        ctx.lcm.handle();
    }

    return 0;
}
