#pragma once

#include "drivers/imu_driver.hpp"
#include "hipnuc_serial.h"

#include <memory>

namespace cerebellum {
namespace drivers {

// Adapter wrapping the vendored, unmodified HiPNUC serial SDK
// (third_party/hipnuc) behind the ImuDriver interface.
// One owner per instance: the wrapped hipnuc_serial_t must not be copied
// or shared between threads (SDK restriction).
class HipnucImuDriver : public ImuDriver {
public:
    ~HipnucImuDriver() override;

    bool init(const core::ImuConfig& config) override;
    int read(ImuData& out, int timeout_ms) override;

private:
    static void convert(const hipnuc_data_t& in, ImuData& out);

    std::unique_ptr<hipnuc_serial_t> dev_;
};

} // namespace drivers
} // namespace cerebellum
