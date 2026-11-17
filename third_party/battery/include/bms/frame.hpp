// Modbus RTU frame building and parsing (read path only; the write path is
// deferred to a later increment).
//
// Frame layout: addr(1) + func(1) + data(n) + crc(2, low byte first).
// RTU has no frame delimiter: the driver reads a response incrementally
// (2-byte header first, then branches on the function code) instead of
// relying on the 3.5-char-time silence, which termios cannot expose.
#pragma once

#include <cstddef>
#include <cstdint>

#include "bms/crc16.hpp"
#include "bms/types.hpp"

namespace bms {

// 43-register response is 91 bytes; 256 leaves headroom.
constexpr std::size_t kMaxFrameLen = 256;

// ---- Request builder ------------------------------------------------------
// 03 read holding registers. Returns the frame length (always 8).
inline std::size_t build_read_request(uint8_t slave, uint16_t start,
                                      uint16_t count, uint8_t* out) {
    out[0] = slave;
    out[1] = 0x03;
    out[2] = static_cast<uint8_t>(start >> 8);
    out[3] = static_cast<uint8_t>(start & 0xFF);
    out[4] = static_cast<uint8_t>(count >> 8);
    out[5] = static_cast<uint8_t>(count & 0xFF);
    append_crc(out, 6);
    return 8;
}

// ---- Response length helpers ----------------------------------------------
// Normal 03 response: addr + func + byte_count + 2*count + crc.
inline std::size_t read_response_len(std::size_t count) {
    return 5 + 2 * count;
}
// Exception response: addr + func|0x80 + exception code + crc.
constexpr std::size_t kExceptionResponseLen = 5;

// ---- Response classification (first two bytes) ----------------------------
enum class ResponseHead { Read, Write, Exception, Unknown };

inline ResponseHead classify_response_head(uint8_t func) {
    if ((func & 0x80u) != 0) {
        return ResponseHead::Exception;
    }
    if (func == 0x03) {
        return ResponseHead::Read;
    }
    if (func == 0x06) {
        return ResponseHead::Write;
    }
    return ResponseHead::Unknown;
}

// ---- Response parsers ------------------------------------------------------
// Parse a complete 03 response and copy register values into out.
inline Error parse_read_response(const uint8_t* frame, std::size_t len,
                                 uint8_t expected_slave,
                                 std::size_t expected_count, uint16_t* out) {
    if (len != read_response_len(expected_count)) {
        return Error::FrameMalformed;
    }
    if (frame[0] != expected_slave) {
        return Error::FrameMalformed;
    }
    if (frame[1] != 0x03) {
        return Error::FrameMalformed;
    }
    if (frame[2] != static_cast<uint8_t>(2 * expected_count)) {
        return Error::FrameMalformed;
    }
    if (!crc16_check(frame, len)) {
        return Error::CrcMismatch;
    }
    for (std::size_t i = 0; i < expected_count; ++i) {
        const std::size_t o = 3 + 2 * i;
        out[i] = static_cast<uint16_t>((static_cast<uint16_t>(frame[o]) << 8) |
                                       frame[o + 1]);
    }
    return Error::Ok;
}

// Parse a complete exception response.
inline Error parse_exception_response(const uint8_t* frame, std::size_t len,
                                      uint8_t expected_slave,
                                      ModbusExceptionCode* code) {
    if (len != kExceptionResponseLen) {
        return Error::FrameMalformed;
    }
    if (frame[0] != expected_slave) {
        return Error::FrameMalformed;
    }
    if ((frame[1] & 0x80u) == 0) {
        return Error::FrameMalformed;
    }
    if (!crc16_check(frame, len)) {
        return Error::CrcMismatch;
    }
    if (code != nullptr) {
        *code = static_cast<ModbusExceptionCode>(frame[2]);
    }
    return Error::ModbusException;
}

}  // namespace bms
