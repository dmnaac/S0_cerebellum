#pragma once

#include "core/state_context.hpp"

#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace cerebellum {
namespace core {

enum class StateType {
    STATIC,
    GAIT
};

class State {
public:
    explicit State(const std::string& name, StateType type);
    virtual ~State() = default;

    const std::string& getName() const;
    StateType getType() const;

    // Called when entering this state
    virtual void onEnter(const StateData& data, const std::string& prev_state) {}

    // Called when leaving this state
    virtual void onExit(const StateData& data, const std::string& next_state) {}

    // Compute state output. Returns true if output is valid.
    virtual bool computeOutput(const StateData& data, StateOutput& output) = 0;

    // Check if transition from this state to target is allowed
    virtual bool canTransitionTo(const std::string& target_state) const;

    // Transition duration in seconds when entering this state
    virtual float getTransitionDuration() const { return 0.5f; }

    // Factory method to create states from YAML config
    static std::unique_ptr<State> createFromConfig(const YAML::Node& config);

protected:
    std::string name_;
    StateType type_;
    std::vector<std::string> allowed_transitions_;
};

} // namespace core
} // namespace cerebellum
