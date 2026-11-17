#include "core/robot_config.hpp"

#include <algorithm>
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

        if (root["motors"]) {
            config.motors.clear();
            for (const auto& m : root["motors"]) {
                MotorConfig mc;
                mc.id = m["id"].as<int>();
                mc.can_iface = m["can_iface"].as<std::string>();
                mc.name = m["name"].as<std::string>();
                mc.protocol = m["protocol"].as<std::string>("robstride");
                mc.master_id = m["master_id"].as<int>(0xFD);
                mc.actuator_type = m["actuator_type"].as<int>(3);
                mc.position_min = m["position_min"].as<double>();
                mc.position_max = m["position_max"].as<double>();
                mc.velocity_max = m["velocity_max"].as<double>();
                mc.torque_max = m["torque_max"].as<double>();
                mc.kp_default = m["kp_default"].as<double>();
                mc.kd_default = m["kd_default"].as<double>();
                config.motors.push_back(mc);
            }
            std::sort(config.motors.begin(), config.motors.end(),
                      [](const MotorConfig& a, const MotorConfig& b) { return a.id < b.id; });
        }

        if (root["default_stand_position"]) {
            const auto& stand = root["default_stand_position"].as<std::vector<double>>();
            config.default_stand_position = Eigen::Map<const Eigen::VectorXd>(stand.data(), stand.size());
        }

        if (root["imu"]) {
            const auto& imu = root["imu"];
            config.imu.protocol = imu["protocol"].as<std::string>(config.imu.protocol);
            config.imu.port = imu["port"].as<std::string>(config.imu.port);
            config.imu.baud = imu["baud"].as<int>(config.imu.baud);
            config.imu.read_timeout_ms = imu["read_timeout_ms"].as<int>(config.imu.read_timeout_ms);
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load robot config: " << e.what() << std::endl;
        return false;
    }
}

} // namespace core
} // namespace cerebellum
