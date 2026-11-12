#include "drivers/joint_motor.hpp"
#include "drivers/robstride_motor_adapter.hpp"

namespace cerebellum {
namespace drivers {

std::unique_ptr<JointMotor> createMotor(const core::MotorConfig& config) {
    if (config.protocol == "robstride") {
        return std::make_unique<RobStrideMotorAdapter>();
    }
    // Future vendors: add another branch here, e.g.
    // if (config.protocol == "unitree") return std::make_unique<UnitreeMotorAdapter>();
    return nullptr;
}

} // namespace drivers
} // namespace cerebellum
