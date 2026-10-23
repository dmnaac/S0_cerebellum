#pragma once

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

class JointMotor {
public:
    virtual ~JointMotor() = default;

    virtual bool init(int id, const std::string& can_iface) = 0;
    virtual bool enable() = 0;
    virtual bool disable() = 0;
    virtual bool setCommand(double pos, double vel, double kp, double kd, double torque) = 0;
    virtual bool readState(JointState& state) = 0;
};

// TODO: add concrete implementations like UnitreeMotor, DMMotor, etc.

} // namespace drivers
} // namespace cerebellum
