#include "drivers/robstride_motor_adapter.hpp"

namespace cerebellum {
namespace drivers {

bool RobStrideMotorAdapter::init(const core::MotorConfig& config) {
    // RobStrideMotor opens its own SocketCAN socket in the constructor
    // (it calls exit(1) on failure — config must be validated beforehand).
    motor_ = std::make_unique<RobStrideMotor>(
        config.can_iface,
        static_cast<uint8_t>(config.master_id),
        static_cast<uint8_t>(config.id),
        config.actuator_type);
    return motor_ != nullptr;
}

bool RobStrideMotorAdapter::enable() {
    if (!motor_) return false;
    storeFeedback(motor_->enable_motor());
    return true;
}

bool RobStrideMotorAdapter::disable() {
    if (!motor_) return false;
    motor_->Disenable_Motor(0);
    return true;
}

bool RobStrideMotorAdapter::setCommand(double pos, double vel, double kp, double kd,
                                       double torque) {
    if (!motor_) return false;
    // RobStrideMotor argument order is (torque, position, velocity, kp, kd);
    // the feedback frame arrives synchronously as the return value.
    storeFeedback(motor_->send_motion_command(
        static_cast<float>(torque),
        static_cast<float>(pos),
        static_cast<float>(vel),
        static_cast<float>(kp),
        static_cast<float>(kd)));
    return true;
}

bool RobStrideMotorAdapter::readState(JointState& state) {
    state = last_state_;
    return true;
}

void RobStrideMotorAdapter::storeFeedback(
    const std::tuple<float, float, float, float>& fb) {
    last_state_.position = std::get<0>(fb);
    last_state_.velocity = std::get<1>(fb);
    last_state_.torque = std::get<2>(fb);
    last_state_.temperature = std::get<3>(fb);
    last_state_.error = motor_ ? static_cast<int16_t>(motor_->fault_code_ & 0xFFFF) : 0;
}

} // namespace drivers
} // namespace cerebellum
