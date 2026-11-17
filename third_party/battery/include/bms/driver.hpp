// BmsDriver: transaction management on top of ITransport (read path).
//
// Tunables are compile-time constants; promote them to runtime parameters
// only after hardware bring-up confirms the values.
#pragma once

#include <cstdint>
#include <memory>
#include <mutex>

#include "bms/transport.hpp"
#include "bms/types.hpp"

namespace bms {

constexpr int kChunkSize = 20;          // registers per 03 frame (spec leaves the max undefined)
constexpr int kRetries = 2;             // attempts per transaction on timeout/CRC failure
constexpr int kTimeoutMs = 500;         // 43-reg response ~95ms @9600 8N1
constexpr int kInterFrameDelayMs = 5;   // >= 3.5 char times (~3.7ms @9600)

class BmsDriver {
public:
    // Precondition: transport must not be null.
    BmsDriver(std::unique_ptr<ITransport> transport, uint8_t slave = 1);

    // Read count registers starting at start into out (raw values).
    // Splits into kChunkSize chunks; retries per transaction on
    // timeout/CRC/frame errors (not on slave exceptions).
    Error read_registers(uint16_t start, uint16_t count, uint16_t* out);

    // Read protection parameters (registers 0~17).
    Error read_parameters(ProtectionParams* out);

    // Read live status (registers 18~42), converted to engineering units.
    Error read_status(BatteryStatus* out);

    // Full snapshot: parameters (0~17) + status (18~42).
    Error read_all(BatteryStatus* status, ProtectionParams* params);

    // Diagnostics.
    Error last_error() const { return last_error_; }
    ModbusExceptionCode last_exception() const { return last_exception_; }
    void set_verbose(bool on) { verbose_ = on; }
    uint8_t slave() const { return slave_; }

private:
    Error transaction_read(uint16_t start, uint16_t count, uint16_t* out);
    void log_frame(const char* tag, const uint8_t* data, std::size_t len) const;

    std::unique_ptr<ITransport> transport_;
    uint8_t slave_;
    Error last_error_ = Error::Ok;
    ModbusExceptionCode last_exception_{};
    bool verbose_ = false;
    mutable std::mutex mutex_;
};

}  // namespace bms
