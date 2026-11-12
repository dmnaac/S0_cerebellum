#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

namespace cerebellum {
namespace core {

struct MotorConfig {
    int id;
    std::string can_iface;
    std::string name;
    std::string protocol;   // motor driver protocol, e.g. "robstride"
    int master_id = 0xFD;   // host CAN ID on the bus
    int actuator_type = 3;  // RobStride actuator model (3 = RS03)
    double position_min;
    double position_max;
    double velocity_max;
    double torque_max;
    double kp_default;
    double kd_default;
};

struct RobotConfig {
    std::vector<MotorConfig> motors;
    Eigen::VectorXd default_stand_position;
    double control_frequency;
    double policy_frequency;
    double max_roll;
    double max_pitch;
    uint64_t comm_timeout_us;
};

bool loadRobotConfig(const std::string& filepath, RobotConfig& config);

} // namespace core
} // namespace cerebellum
