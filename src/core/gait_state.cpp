#include "core/gait_state.hpp"

#include <iostream>

namespace cerebellum {
namespace core {

GaitState::GaitState(const std::string& name, int gait_id,
                     const std::string& policy_name,
                     const Eigen::VectorXf& default_action,
                     float transition_duration)
    : State(name, StateType::GAIT),
      gait_id_(gait_id),
      policy_name_(policy_name),
      default_action_(default_action),
      transition_duration_(transition_duration) {}

void GaitState::setPolicyModel(rl::PolicyModel* model) {
    policy_model_ = model;
}

bool GaitState::computeOutput(const StateData& data, StateOutput& output) {
    output.desired_velocity = Eigen::VectorXf::Zero(12);
    output.desired_torque = Eigen::VectorXf::Zero(12);

    if (policy_model_ == nullptr) {
        // Fallback: use default action offset added to default stand position
        output.action = data.default_position + default_action_;
        output.desired_position = output.action;
        output.is_valid = true;
        return true;
    }

    Eigen::VectorXf obs = buildObservation(data);
    Eigen::VectorXf action_offset;
    if (policy_model_->inference(obs, action_offset)) {
        output.action = data.default_position + action_offset;
        output.desired_position = output.action;
        output.is_valid = true;
        return true;
    }

    output.action = data.default_position + default_action_;
    output.desired_position = output.action;
    output.is_valid = true;
    return true;
}

Eigen::VectorXf GaitState::buildObservation(const StateData& data) {
    // Build observation consistent with RL training
    // Includes gait_id as one-hot or scalar input for multi-gait policies
    int obs_dim = 3 + 3 + 3 + 12 + 12 + 12 + 1; // ~46
    Eigen::VectorXf obs(obs_dim);
    int idx = 0;

    // Projected gravity
    for (int i = 0; i < 3; ++i) obs(idx++) = data.gravity_vector(i);

    // Command
    for (int i = 0; i < 3; ++i) obs(idx++) = data.command(i);

    // Angular velocity
    for (int i = 0; i < 3; ++i) obs(idx++) = data.angular_velocity(i);

    // Joint positions
    for (int i = 0; i < 12; ++i) obs(idx++) = data.joint_position(i);

    // Joint velocities
    for (int i = 0; i < 12; ++i) obs(idx++) = data.joint_velocity(i);

    // Previous actions
    for (int i = 0; i < 12; ++i) obs(idx++) = data.previous_action(i);

    // Gait ID
    obs(idx++) = static_cast<float>(gait_id_);

    return obs;
}

std::unique_ptr<State> GaitState::createFromConfig(const YAML::Node& config) {
    std::string name = config["name"].as<std::string>();
    int gait_id = config["gait_id"].as<int>();
    std::string policy_name = config["policy"].as<std::string>("default");
    float transition = config["transition_duration"].as<float>(0.3f);

    Eigen::VectorXf default_action = Eigen::VectorXf::Zero(12);
    if (config["default_action"]) {
        auto vec = config["default_action"].as<std::vector<float>>();
        for (size_t i = 0; i < vec.size() && i < 12; ++i) {
            default_action(i) = vec[i];
        }
    }

    return std::make_unique<GaitState>(name, gait_id, policy_name, default_action, transition);
}

} // namespace core
} // namespace cerebellum
