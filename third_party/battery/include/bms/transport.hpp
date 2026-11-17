// Transport abstraction.
//
// Read semantics: read exactly len bytes, or return false when the timeout
// expires first. Implementations: PosixSerial (RK3588/Ubuntu, serial_posix.cpp),
// FakeTransport (tests). Serial parameters are fixed by the spec: 9600 8N1.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

namespace bms {

class ITransport {
public:
    virtual ~ITransport() = default;
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool is_open() const = 0;
    virtual bool write(const uint8_t* data, std::size_t len) = 0;
    virtual bool read_exact(uint8_t* data, std::size_t len, int timeout_ms) = 0;
};

// POSIX serial transport factory (Linux/RK3588), implemented in
// serial_posix.cpp; not built on non-POSIX platforms.
// direction: "auto" (default) or "kernel" (kernel RS485 mode via TIOCSRS485).
std::unique_ptr<ITransport> make_posix_serial(const char* device, int baud,
                                              const char* direction,
                                              uint32_t rts_before_send_us,
                                              uint32_t rts_after_send_us);

}  // namespace bms
