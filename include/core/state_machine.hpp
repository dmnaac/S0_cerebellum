#pragma once

#include "core/state.hpp"
#include "core/state_context.hpp"
#include "rl/policy_model.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace cerebellum {
namespace core {

class StateMachine {
public:
    explicit StateMachine(const YAML::Node& config);

    // Load state definitions from config
    bool loadStates();

    // Register a policy model for gait states
    void registerPolicy(const std::string& name, std::unique_ptr<rl::PolicyModel> model);

    // Request state transition
    bool requestState(const std::string& state_name, const StateData& data);

    // Get current state name
    std::string getCurrentState() const;

    // Check if currently in transition
    bool isTransitioning() const;

    // Main update: computes output for current state, handles transitions
    bool update(const StateData& data, StateOutput& output);

    // Get list of available states
    std::vector<std::string> getAvailableStates() const;

private:
    YAML::Node config_;
    std::map<std::string, std::unique_ptr<State>> states_;
    std::map<std::string, std::unique_ptr<rl::PolicyModel>> policies_;

    std::string current_state_name_;
    std::string previous_state_name_;
    bool in_transition_ = false;
    float transition_time_ = 0.0f;
    float transition_duration_ = 0.0f;
    StateOutput previous_output_;
    StateOutput target_output_;

    bool validateConfig() const;
    State* getState(const std::string& name);
};

} // namespace core
} // namespace cerebellum
