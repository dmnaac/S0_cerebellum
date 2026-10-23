#pragma once

#include "core/state.hpp"

#include <Eigen/Core>

namespace cerebellum {
namespace core {

// Static posture state: stand, crouch, lie, lock, etc.
class StaticState : public State {
public:
    StaticState(const std::string& name, const Eigen::VectorXf& target_position,
                float kp, float kd, float transition_duration);

    bool computeOutput(const StateData& data, StateOutput& output) override;

    float getTransitionDuration() const override { return transition_duration_; }

    static std::unique_ptr<State> createFromConfig(const YAML::Node& config);

private:
    Eigen::VectorXf target_position_;  // 12-dim absolute joint target
    float kp_;
    float kd_;
    float transition_duration_;
};

} // namespace core
} // namespace cerebellum
