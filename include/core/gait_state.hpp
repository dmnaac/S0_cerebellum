#pragma once

#include "core/state.hpp"
#include "rl/policy_model.hpp"

#include <memory>
#include <string>

namespace cerebellum {
namespace core {

// Dynamic gait state: crawl, walk, trot, pace, bound, gallop
class GaitState : public State {
public:
    GaitState(const std::string& name, int gait_id,
              const std::string& policy_name,
              const Eigen::VectorXf& default_action,
              float transition_duration);

    bool computeOutput(const StateData& data, StateOutput& output) override;

    float getTransitionDuration() const override { return transition_duration_; }

    // Set the policy model (lazy load by StateMachine)
    void setPolicyModel(rl::PolicyModel* model);

    static std::unique_ptr<State> createFromConfig(const YAML::Node& config);

private:
    int gait_id_;
    std::string policy_name_;
    Eigen::VectorXf default_action_;
    float transition_duration_;
    rl::PolicyModel* policy_model_ = nullptr;

    Eigen::VectorXf buildObservation(const StateData& data);
};

} // namespace core
} // namespace cerebellum
