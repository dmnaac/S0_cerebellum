#include "core/state_machine.hpp"

#include <algorithm>
#include <iostream>
#include <vector>

namespace cerebellum {
namespace core {

StateMachine::StateMachine(const YAML::Node& config) : config_(config) {
    loadStates();
}

bool StateMachine::loadStates() {
    if (!validateConfig()) {
        return false;
    }

    const auto& states = config_["states"];
    for (const auto& s : states) {
        auto state = State::createFromConfig(s);
        if (state) {
            states_[state->getName()] = std::move(state);
        } else {
            std::cerr << "Failed to create state from config" << std::endl;
            return false;
        }
    }

    // Set initial state
    std::string initial = config_["initial_state"].as<std::string>("stand");
    if (states_.find(initial) != states_.end()) {
        current_state_name_ = initial;
    } else if (!states_.empty()) {
        current_state_name_ = states_.begin()->first;
    }

    return true;
}

bool StateMachine::validateConfig() const {
    if (!config_["states"]) {
        std::cerr << "StateMachine config missing 'states'" << std::endl;
        return false;
    }
    return true;
}

void StateMachine::registerPolicy(const std::string& name, std::unique_ptr<rl::PolicyModel> model) {
    policies_[name] = std::move(model);

    // Assign policy model to gait states that reference it
    for (auto& [state_name, state] : states_) {
        if (state->getType() == StateType::GAIT) {
            auto* gait_state = dynamic_cast<GaitState*>(state.get());
            if (gait_state) {
                // For simplicity, all gait states use registered policies by name
                // In a real system, store policy name in GaitState and match here
                gait_state->setPolicyModel(policies_[name].get());
            }
        }
    }
}

bool StateMachine::requestState(const std::string& state_name, const StateData& data) {
    if (state_name == current_state_name_) {
        return true;
    }

    State* current = getState(current_state_name_);
    if (!current) {
        std::cerr << "Current state not found: " << current_state_name_ << std::endl;
        return false;
    }

    if (!current->canTransitionTo(state_name)) {
        std::cerr << "Transition from " << current_state_name_ << " to "
                  << state_name << " not allowed" << std::endl;
        return false;
    }

    State* target = getState(state_name);
    if (!target) {
        std::cerr << "Target state not found: " << state_name << std::endl;
        return false;
    }

    // Start transition
    previous_state_name_ = current_state_name_;
    current->onExit(data, state_name);
    target->onEnter(data, current_state_name_);

    current_state_name_ = state_name;
    in_transition_ = true;
    transition_time_ = 0.0f;
    transition_duration_ = target->getTransitionDuration();

    return true;
}

std::string StateMachine::getCurrentState() const {
    return current_state_name_;
}

bool StateMachine::isTransitioning() const {
    return in_transition_;
}

bool StateMachine::update(const StateData& data, StateOutput& output) {
    State* current = getState(current_state_name_);
    if (!current) {
        std::cerr << "No current state" << std::endl;
        return false;
    }

    if (!current->computeOutput(data, target_output_)) {
        return false;
    }

    if (in_transition_) {
        transition_time_ += data.dt;
        float alpha = std::min(1.0f, transition_time_ / transition_duration_);

        // Smooth interpolation (ease-in-out)
        float smooth_alpha = alpha * alpha * (3.0f - 2.0f * alpha);

        output.action = (1.0f - smooth_alpha) * previous_output_.action +
                        smooth_alpha * target_output_.action;
        output.desired_position = (1.0f - smooth_alpha) * previous_output_.desired_position +
                                  smooth_alpha * target_output_.desired_position;
        output.desired_velocity = target_output_.desired_velocity;
        output.desired_torque = target_output_.desired_torque;
        output.is_valid = true;

        if (alpha >= 1.0f) {
            in_transition_ = false;
            previous_output_ = target_output_;
        }
    } else {
        output = target_output_;
        previous_output_ = target_output_;
    }

    return true;
}

std::vector<std::string> StateMachine::getAvailableStates() const {
    std::vector<std::string> names;
    for (const auto& [name, state] : states_) {
        names.push_back(name);
    }
    return names;
}

State* StateMachine::getState(const std::string& name) {
    auto it = states_.find(name);
    if (it != states_.end()) {
        return it->second.get();
    }
    return nullptr;
}

} // namespace core
} // namespace cerebellum
