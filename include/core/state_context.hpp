#pragma once

#include <Eigen/Core>
#include <cstdint>

namespace cerebellum {
namespace core {

// Forward declarations
struct StateData {
    Eigen::VectorXf joint_position;
    Eigen::VectorXf joint_velocity;
    Eigen::Vector3f gravity_vector;  // projected gravity in body frame
    Eigen::Vector3f angular_velocity;
    Eigen::Vector3f command;         // [vx, vy, yaw_rate]
    Eigen::VectorXf previous_action; // 12-dim offset from default stand
    Eigen::VectorXf default_position; // 12-dim default stand position
    float dt;
    int64_t utime;
};

struct StateOutput {
    Eigen::VectorXf action;          // 12-dim absolute target joint position
    Eigen::VectorXf desired_position;
    Eigen::VectorXf desired_velocity;
    Eigen::VectorXf desired_torque;
    bool is_valid;
};

} // namespace core
} // namespace cerebellum
