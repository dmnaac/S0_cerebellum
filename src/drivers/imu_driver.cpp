#include "drivers/imu_driver.hpp"
#include "drivers/hipnuc_imu_driver.hpp"

namespace cerebellum {
namespace drivers {

std::unique_ptr<ImuDriver> createImuDriver(const core::ImuConfig& config) {
    if (config.protocol == "hipnuc") {
        return std::make_unique<HipnucImuDriver>();
    }
    // Future vendors: add another branch here, e.g.
    // if (config.protocol == "wit") return std::make_unique<WitImuDriver>();
    return nullptr;
}

} // namespace drivers
} // namespace cerebellum
