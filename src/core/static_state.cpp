#include "core/static_state.hpp"

#include <iostream>

namespace cerebellum {
namespace core {

StaticState::StaticState(const std::string& name, const Eigen::VectorXf& target_position,
                         float kp, float kd, float transition_duration)
    : State(name, StateType::STATIC),
      target_position_(target_position),
      kp_(kp),
      kd_(kd),
      transition_duration_(transition_duration) {}

bool StaticState::computeOutput(const StateData& data, StateOutput& output) {
    output.desired_position = target_position_;
    output.desired_velocity = Eigen::VectorXf::Zero(12);
    output.desired_torque = Eigen::VectorXf::Zero(12);

    // For static states, action is the absolute target position offset from default stand
    // This will be interpreted by motion_control_node as target position
    output.action = target_position_;
    output.is_valid = true;
    return true;
}

std::unique_ptr<State> StaticState::createFromConfig(const YAML::Node& config) {
    std::string name = config["name"].as<std::string>();
    auto pos_vec = config["target_position"].as<std::vector<float>>();
    Eigen::VectorXf target(pos_vec.size());
    for (size_t i = 0; i < pos_vec.size(); ++i) {
        target(i) = pos_vec[i];
    }
    float kp = config["kp"].as<float>(50.0f);
    float kd = config["kd"].as<float>(2.0f);
    float transition = config["transition_duration"].as<float>(0.5f);

    return std::make_unique<StaticState>(name, target, kp, kd, transition);
}

} // namespace core
} // namespace cerebellum
