#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

namespace cerebellum {
namespace rl {

class PolicyModel {
public:
    virtual ~PolicyModel() = default;

    virtual bool load(const std::string& model_path) = 0;
    virtual bool inference(const Eigen::VectorXf& observation, Eigen::VectorXf& action) = 0;

    int getObservationDim() const;
    int getActionDim() const;

protected:
    int obs_dim_ = 0;
    int action_dim_ = 0;
};

// TODO: add OnnxPolicy, RknnPolicy implementations

} // namespace rl
} // namespace cerebellum
