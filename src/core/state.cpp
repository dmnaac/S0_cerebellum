#include "core/state.hpp"
#include "core/gait_state.hpp"
#include "core/static_state.hpp"

namespace cerebellum {
namespace core {

State::State(const std::string& name, StateType type) : name_(name), type_(type) {}

const std::string& State::getName() const {
    return name_;
}

StateType State::getType() const {
    return type_;
}

bool State::canTransitionTo(const std::string& target_state) const {
    if (allowed_transitions_.empty()) {
        return true; // Allow all by default
    }
    for (const auto& s : allowed_transitions_) {
        if (s == target_state) return true;
    }
    return false;
}

std::unique_ptr<State> State::createFromConfig(const YAML::Node& config) {
    std::string type = config["type"].as<std::string>();
    if (type == "static") {
        return StaticState::createFromConfig(config);
    } else if (type == "gait") {
        return GaitState::createFromConfig(config);
    }
    return nullptr;
}

} // namespace core
} // namespace cerebellum
