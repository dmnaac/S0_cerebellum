#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace cerebellum {
namespace core {
struct ImuConfig;
}
namespace drivers {

// Hardware-independent IMU sample. Each field is meaningful only when its
// valid bit is set. Orientation convention follows the device output
// (see hipnuc_data.h COORD notes); the adapter passes axes through unchanged.
struct ImuData {
    uint64_t valid;  // IMU_VALID_* bits below

    float quat[4];   // w, x, y, z
    float gyro[3];   // rad/s
    float accel[3];  // m/s^2, specific force (gravity not removed)
    float roll;      // rad
    float pitch;     // rad
    float yaw;       // rad
    float temperature;
    uint16_t main_status;  // device status bits (vendor specific)
};

#define IMU_VALID_QUAT (uint64_t(1) << 0)
#define IMU_VALID_GYRO (uint64_t(1) << 1)
#define IMU_VALID_ACCEL (uint64_t(1) << 2)
#define IMU_VALID_RPY (uint64_t(1) << 3)
#define IMU_VALID_TEMPERATURE (uint64_t(1) << 4)
#define IMU_VALID_STATUS (uint64_t(1) << 5)

// Hardware-independent IMU interface. sensor_node only talks to this
// interface, so swapping IMU vendors only requires a new adapter plus a
// factory branch in createImuDriver().
class ImuDriver {
public:
    virtual ~ImuDriver() = default;

    virtual bool init(const core::ImuConfig& config) = 0;

    // Blocking read with timeout. Returns:
    //   1  new sample stored in `out`
    //   0  no sample within timeout_ms
    //  -1  device/port error (implementation may expose details via log)
    virtual int read(ImuData& out, int timeout_ms) = 0;
};

// Creates an IMU driver instance based on config.protocol.
// Returns nullptr for unsupported protocols.
std::unique_ptr<ImuDriver> createImuDriver(const core::ImuConfig& config);

} // namespace drivers
} // namespace cerebellum
