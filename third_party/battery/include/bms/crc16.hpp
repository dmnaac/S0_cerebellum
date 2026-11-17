// Modbus CRC16 for the BMS RS485 link.
//
// Implementation reference: spec Appendix B method 1 (bitwise), polynomial
// 0xA001 (reflected x^16 + x^15 + x^2 + 1), init 0xFFFF, no final XOR.
// Verified against the canonical check value crc16("123456789") == 0x4B37
// and the spec example frame 01 03 00 01 00 02 -> 0xCB95 (transmitted
// 95 CB, low byte first).
//
// The spec's Appendix B method-2 listing is deliberately NOT used as the
// reference: it contains typos ("newbyte", two transpositions in the
// auchCRCLo row 14) and its printed return statement yields a byte-swapped
// CRC (measured on 1000 random inputs: all disagree with the bitwise method
// until the return bytes are exchanged). The 256-entry table below is
// generated at compile time from the bitwise method instead, so there is
// nothing to hand-transcribe.
#pragma once

#include <cstddef>
#include <cstdint>

namespace bms {

constexpr uint16_t kCrc16Init = 0xFFFF;

// One bitwise step over a single byte.
constexpr uint16_t crc16_step(uint16_t crc, uint8_t byte) {
    crc = static_cast<uint16_t>(crc ^ byte);
    for (int i = 0; i < 8; ++i) {
        crc = (crc & 1u) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001u)
                         : static_cast<uint16_t>(crc >> 1);
    }
    return crc;
}

// Bitwise CRC16 (constexpr: usable in static_assert).
constexpr uint16_t crc16(const uint8_t* data, std::size_t len) {
    uint16_t crc = kCrc16Init;
    for (std::size_t i = 0; i < len; ++i) {
        crc = crc16_step(crc, data[i]);
    }
    return crc;
}

// Compile-time generated 256-entry table; equivalent to the bitwise version
// (cross-checked in tests over pseudo-random data).
struct Crc16Table {
    uint16_t entry[256];

    constexpr Crc16Table() : entry{} {
        for (int i = 0; i < 256; ++i) {
            uint16_t crc = static_cast<uint16_t>(i);
            for (int b = 0; b < 8; ++b) {
                crc = (crc & 1u) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001u)
                                 : static_cast<uint16_t>(crc >> 1);
            }
            entry[i] = crc;
        }
    }
};

inline constexpr Crc16Table kCrc16Table{};

// Table-driven CRC16 (same result as crc16(); irrelevant at 9600bps but
// free). crc = (crc >> 8) ^ table[(crc ^ byte) & 0xFF].
inline uint16_t crc16_fast(const uint8_t* data, std::size_t len) {
    uint16_t crc = kCrc16Init;
    for (std::size_t i = 0; i < len; ++i) {
        crc = static_cast<uint16_t>((crc >> 8) ^
                                    kCrc16Table.entry[(crc ^ data[i]) & 0xFFu]);
    }
    return crc;
}

// Append the CRC to a frame: low byte first (CRCLO, CRCHI), per spec.
inline void append_crc(uint8_t* frame, std::size_t len_without_crc) {
    const uint16_t crc = crc16_fast(frame, len_without_crc);
    frame[len_without_crc] = static_cast<uint8_t>(crc & 0xFFu);
    frame[len_without_crc + 1] = static_cast<uint8_t>(crc >> 8);
}

// Verify a received frame's CRC (frame includes the two CRC bytes).
inline bool crc16_check(const uint8_t* frame, std::size_t len) {
    if (len < 3) {
        return false;
    }
    const uint16_t received =
        static_cast<uint16_t>(static_cast<uint16_t>(frame[len - 2]) |
                              (static_cast<uint16_t>(frame[len - 1]) << 8));
    return crc16_fast(frame, len - 2) == received;
}

}  // namespace bms
