#include <lcm/lcm-cpp.hpp>
#include <quad/robot_state_t.hpp>
#include <quad/rl_action_t.hpp>
#include <quad/state_status_t.hpp>
#include <quad/user_cmd_t.hpp>

#include "core/robot_config.hpp"
#include "core/state_context.hpp"
#include "core/state_machine.hpp"
#include "rl/policy_model.hpp"
#include "utils/config_parser.hpp"

#include <Eigen/Core>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

// TODO: implement concrete PolicyModel (OnnxPolicy / RknnPolicy)

struct PolicyContext {
    lcm::LCM lcm;
    quad::robot_state_t robot_state;
    quad::user_cmd_t latest_cmd;
    quad::rl_action_t action;
    quad::state_status_t state_status;

    std::unique_ptr<cerebellum::core::StateMachine> state_machine;
    cerebellum::core::StateData state_data;
    cerebellum::core::RobotConfig robot_config;

    bool state_ready = false;
    bool cmd_ready = false;
    int64_t last_update_time = 0;
};

void robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                       const quad::robot_state_t* msg, PolicyContext* ctx) {
    ctx->robot_state = *msg;
    ctx->state_ready = true;
}

void userCmdHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                    const quad::user_cmd_t* msg, PolicyContext* ctx) {
    ctx->latest_cmd = *msg;
    ctx->cmd_ready = true;

    // Request state transition based on user command
    if (!msg->state_name.empty() && ctx->state_machine) {
        ctx->state_machine->requestState(msg->state_name, ctx->state_data);
    }
}

void updateStateData(PolicyContext& ctx) {
    auto& data = ctx.state_data;

    data.joint_position = Eigen::Map<const Eigen::VectorXf>(
        ctx.robot_state.joints.position, 12);
    data.joint_velocity = Eigen::Map<const Eigen::VectorXf>(
        ctx.robot_state.joints.velocity, 12);

    // Projected gravity: assume identity rotation for placeholder
    data.gravity_vector = Eigen::Vector3f(
        0.0f, 0.0f, -1.0f);

    data.angular_velocity = Eigen::Map<const Eigen::Vector3f>(
        ctx.robot_state.imu.gyro);
    data.command = Eigen::Vector3f(
        ctx.latest_cmd.vx, ctx.latest_cmd.vy, ctx.latest_cmd.yaw_rate);
    data.previous_action = Eigen::Map<const Eigen::VectorXf>(
        ctx.robot_state.previous_action, 12);
    data.default_position = ctx.robot_config.default_stand_position.cast<float>();

    auto now = std::chrono::steady_clock::now();
    int64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                         now.time_since_epoch())
                         .count();
    data.utime = now_us;
    data.dt = ctx.last_update_time > 0
                  ? static_cast<float>(now_us - ctx.last_update_time) / 1e6f
                  : 0.02f;
    ctx.last_update_time = now_us;
}

void publishAction(PolicyContext& ctx, const cerebellum::core::StateOutput& output) {
    ctx.action.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                           std::chrono::steady_clock::now().time_since_epoch())
                           .count();
    for (int i = 0; i < 12; ++i) {
        ctx.action.position_offset[i] = output.action(i);
    }
    ctx.lcm.publish("RL_ACTION", &ctx.action);
}

void publishStateStatus(PolicyContext& ctx) {
    if (!ctx.state_machine) return;

    ctx.state_status.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now().time_since_epoch())
                                 .count();
    ctx.state_status.current_state = ctx.state_machine->getCurrentState();
    ctx.state_status.target_state = ctx.state_machine->getCurrentState();
    ctx.state_status.in_transition = ctx.state_machine->isTransitioning();
    ctx.state_status.transition_progress = ctx.state_machine->isTransitioning() ? 0.5f : 1.0f;
    ctx.lcm.publish("STATE_STATUS", &ctx.state_status);
}

int main(int argc, char** argv) {
    PolicyContext ctx;
    if (!ctx.lcm.good()) {
        std::cerr << "rl_policy_node: LCM init failed" << std::endl;
        return -1;
    }

    // Load state machine config
    std::string states_config = "config/states.yaml";
    std::string robot_config = "config/robot.yaml";
    if (argc > 1) {
        states_config = argv[1];
    }
    if (argc > 2) {
        robot_config = argv[2];
    }

    cerebellum::utils::ConfigParser parser(states_config);
    if (!parser.load()) {
        std::cerr << "rl_policy_node: failed to load " << states_config << std::endl;
        return -1;
    }

    if (!cerebellum::core::loadRobotConfig(robot_config, ctx.robot_config)) {
        std::cerr << "rl_policy_node: failed to load " << robot_config << std::endl;
        return -1;
    }

    ctx.state_machine = std::make_unique<cerebellum::core::StateMachine>(parser.getRoot());
    std::cout << "rl_policy_node: current state = " << ctx.state_machine->getCurrentState() << std::endl;

    // TODO: load RL policy model(s) and register with state machine
    // auto policy = std::make_unique<OnnxPolicy>();
    // policy->load("models/default_policy.onnx");
    // ctx.state_machine->registerPolicy("default", std::move(policy));

    ctx.lcm.subscribe("ROBOT_STATE", &robotStateHandler, &ctx);
    ctx.lcm.subscribe("USER_CMD", &userCmdHandler, &ctx);

    std::cout << "rl_policy_node started" << std::endl;

    while (true) {
        auto start = std::chrono::steady_clock::now();

        ctx.lcm.handleTimeout(0);

        if (ctx.state_ready && ctx.cmd_ready) {
            updateStateData(ctx);

            cerebellum::core::StateOutput output;
            if (ctx.state_machine->update(ctx.state_data, output)) {
                publishAction(ctx, output);
                publishStateStatus(ctx);
            }
        }

        std::this_thread::sleep_until(start + std::chrono::milliseconds(20)); // 50Hz
    }

    return 0;
}
