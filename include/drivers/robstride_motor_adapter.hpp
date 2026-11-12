#pragma once

#include "drivers/joint_motor.hpp"
#include "motor_ros2/motor_cfg.h"

#include <memory>
#include <tuple>

namespace cerebellum {
namespace drivers {

// Adapter that wraps the vendored, unmodified RobStrideMotor class
// (third_party/robstride) behind the JointMotor interface.
// It only maps parameter order and caches the synchronous feedback
// returned by RobStride command calls.
class RobStrideMotorAdapter : public JointMotor {
public:
    bool init(const core::MotorConfig& config) override;
    bool enable() override;
    bool disable() override;
    bool setCommand(double pos, double vel, double kp, double kd, double torque) override;
    bool readState(JointState& state) override;

private:
    void storeFeedback(const std::tuple<float, float, float, float>& fb);

    std::unique_ptr<RobStrideMotor> motor_;
    JointState last_state_{};
};

} // namespace drivers
} // namespace cerebellum
