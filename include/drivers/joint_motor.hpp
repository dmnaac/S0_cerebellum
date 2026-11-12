#pragma once

#include "core/robot_config.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace cerebellum {
namespace drivers {

struct JointState {
    double position;
    double velocity;
    double torque;
    double temperature;
    int16_t error;
};

// Hardware-independent motor interface. Concrete adapters (e.g. RobStride)
// live in this directory; motor_driver_node only talks to this interface,
// so swapping motor vendors only requires a new adapter + factory branch.
class JointMotor {
public:
    virtual ~JointMotor() = default;

    virtual bool init(const core::MotorConfig& config) = 0;
    virtual bool enable() = 0;
    virtual bool disable() = 0;
    virtual bool setCommand(double pos, double vel, double kp, double kd, double torque) = 0;
    virtual bool readState(JointState& state) = 0;
};

// Creates a motor driver instance based on config.protocol.
// Returns nullptr for unsupported protocols.
std::unique_ptr<JointMotor> createMotor(const core::MotorConfig& config);

} // namespace drivers
} // namespace cerebellum
