#include "core/robot_config.hpp"

#include <iostream>
#include <yaml-cpp/yaml.h>

namespace cerebellum {
namespace core {

bool loadRobotConfig(const std::string& filepath, RobotConfig& config) {
    try {
        YAML::Node root = YAML::LoadFile(filepath);

        config.control_frequency = root["control_frequency"].as<double>(1000.0);
        config.policy_frequency = root["policy_frequency"].as<double>(50.0);
        config.max_roll = root["max_roll"].as<double>(0.785);
        config.max_pitch = root["max_pitch"].as<double>(0.785);
        config.comm_timeout_us = root["comm_timeout_us"].as<uint64_t>(50000);

        const auto& motors = root["motors"];
        config.motors.clear();
        for (const auto& m : motors) {
            MotorConfig mc;
            mc.id = m["id"].as<int>();
            mc.can_iface = m["can_iface"].as<std::string>();
            mc.name = m["name"].as<std::string>();
            mc.position_min = m["position_min"].as<double>();
            mc.position_max = m["position_max"].as<double>();
            mc.velocity_max = m["velocity_max"].as<double>();
            mc.torque_max = m["torque_max"].as<double>();
            mc.kp_default = m["kp_default"].as<double>();
            mc.kd_default = m["kd_default"].as<double>();
            config.motors.push_back(mc);
        }

        const auto& stand = root["default_stand_position"].as<std::vector<double>>();
        config.default_stand_position = Eigen::Map<const Eigen::VectorXd>(stand.data(), stand.size());

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load robot config: " << e.what() << std::endl;
        return false;
    }
}

} // namespace core
} // namespace cerebellum
