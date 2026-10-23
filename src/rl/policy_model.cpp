#include "rl/policy_model.hpp"

namespace cerebellum {
namespace rl {

int PolicyModel::getObservationDim() const {
    return obs_dim_;
}

int PolicyModel::getActionDim() const {
    return action_dim_;
}

} // namespace rl
} // namespace cerebellum
