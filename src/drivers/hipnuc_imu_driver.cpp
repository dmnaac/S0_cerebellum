#include "drivers/hipnuc_imu_driver.hpp"

#include "core/robot_config.hpp"

#include <iostream>

namespace cerebellum {
namespace drivers {

HipnucImuDriver::~HipnucImuDriver() {
    if (dev_ && dev_->is_open) {
        hipnuc_serial_close(dev_.get());
    }
}

bool HipnucImuDriver::init(const core::ImuConfig& config) {
    dev_ = std::make_unique<hipnuc_serial_t>();
    *dev_ = hipnuc_serial_t{0};  // zero-fill per SDK usage pattern

    if (hipnuc_serial_open(dev_.get(), config.port.c_str(), config.baud) != 0) {
        std::cerr << "hipnuc_imu: failed to open " << config.port << ": "
                  << hipnuc_serial_last_error(dev_.get()) << std::endl;
        return false;
    }

    std::cout << "hipnuc_imu: opened " << config.port << " at " << config.baud
              << " baud" << std::endl;
    return true;
}

int HipnucImuDriver::read(ImuData& out, int timeout_ms) {
    if (!dev_) return -1;

    hipnuc_data_t sample;
    int ret = hipnuc_serial_read(dev_.get(), &sample, timeout_ms);
    if (ret == 1) {
        convert(sample, out);
    }
    return ret;
}

void HipnucImuDriver::convert(const hipnuc_data_t& in, ImuData& out) {
    out.valid = 0;

    if (in.valid & HIPNUC_VALID_QUAT) {
        for (int i = 0; i < 4; ++i) out.quat[i] = in.quat[i];
        out.valid |= IMU_VALID_QUAT;
    }
    if (in.valid & HIPNUC_VALID_GYR) {
        for (int i = 0; i < 3; ++i) out.gyro[i] = in.gyr[i];
        out.valid |= IMU_VALID_GYRO;
    }
    if (in.valid & HIPNUC_VALID_ACC) {
        for (int i = 0; i < 3; ++i) out.accel[i] = in.acc[i];
        out.valid |= IMU_VALID_ACCEL;
    }
    if ((in.valid & HIPNUC_VALID_ROLL_PITCH) && (in.valid & HIPNUC_VALID_YAW)) {
        out.roll = in.roll;
        out.pitch = in.pitch;
        out.yaw = in.yaw;
        out.valid |= IMU_VALID_RPY;
    }
    if (in.valid & HIPNUC_VALID_TEMPERATURE) {
        out.temperature = in.temperature;
        out.valid |= IMU_VALID_TEMPERATURE;
    }
    if (in.valid & HIPNUC_VALID_STATUS) {
        out.main_status = in.main_status;
        out.valid |= IMU_VALID_STATUS;
    }
}

} // namespace drivers
} // namespace cerebellum
