// Register map and unit conversions.
//
// Source: spec Appendix A. Register addresses are used directly as Modbus
// addresses (0-based). The spec's example frame says "starting address 1"
// which leaves a 0/1-based ambiguity; if hardware turns out to be 1-based,
// flip kAddressOffset and rebuild (nothing else changes).
#pragma once

#include <cstddef>
#include <cstdint>

namespace bms {

constexpr uint16_t kAddressOffset = 0;

// ---- Protection parameters (0~17), readable via 03 ----------------------
constexpr uint16_t REG_CELL_OVP = 0;              // mV, single-cell overvoltage
constexpr uint16_t REG_CELL_OVP_RELEASE = 1;      // mV
constexpr uint16_t REG_CELL_UVP = 2;              // mV
constexpr uint16_t REG_CELL_UVP_RELEASE = 3;      // mV
constexpr uint16_t REG_PACK_OVP = 4;              // 10mV
constexpr uint16_t REG_PACK_OVP_RELEASE = 5;      // 10mV
constexpr uint16_t REG_PACK_UVP = 6;              // 10mV
constexpr uint16_t REG_PACK_UVP_RELEASE = 7;      // 10mV
constexpr uint16_t REG_CHG_HIGH_TEMP = 8;         // 0.1K
constexpr uint16_t REG_CHG_HIGH_TEMP_RELEASE = 9;
constexpr uint16_t REG_CHG_LOW_TEMP = 10;
constexpr uint16_t REG_CHG_LOW_TEMP_RELEASE = 11;
constexpr uint16_t REG_DIS_HIGH_TEMP = 12;
constexpr uint16_t REG_DIS_HIGH_TEMP_RELEASE = 13;
constexpr uint16_t REG_DIS_LOW_TEMP = 14;
constexpr uint16_t REG_DIS_LOW_TEMP_RELEASE = 15;
constexpr uint16_t REG_CHG_OCP = 16;              // 10mA
constexpr uint16_t REG_DIS_OCP = 17;              // 10mA
constexpr uint16_t REG_PARAM_FIRST = REG_CELL_OVP;
constexpr uint16_t REG_PARAM_LAST = REG_DIS_OCP;
constexpr uint16_t REG_PARAM_COUNT = REG_PARAM_LAST - REG_PARAM_FIRST + 1;  // 18

// ---- Live data (18~42) ---------------------------------------------------
constexpr uint16_t REG_PACK_VOLTAGE = 18;         // 10mV, up to 655.35V
constexpr uint16_t REG_CURRENT = 19;              // 10mA, signed two's complement
constexpr uint16_t REG_REMAIN_CAPACITY = 20;      // 10mAh
constexpr uint16_t REG_CELL_FIRST = 21;           // mV, cells 1..kCellCount
constexpr std::size_t kCellCount = 16;
constexpr uint16_t REG_CELL_LAST =
    static_cast<uint16_t>(REG_CELL_FIRST + kCellCount - 1);  // 36
constexpr uint16_t REG_TEMP1 = 37;                // 0.1K
constexpr uint16_t REG_TEMP2 = 38;                // 0.1K
constexpr uint16_t REG_NOMINAL_CAPACITY = 39;     // 10mAh
constexpr uint16_t REG_RSOC = 40;                 // low byte = percent
constexpr uint16_t REG_FET = 41;                  // bit0 charge, bit1 discharge
constexpr uint16_t REG_PROTECT_STATUS = 42;       // bit0..12
constexpr uint16_t REG_LAST = REG_PROTECT_STATUS;  // 42

// ---- Conversions (spec notes 1~4) ----------------------------------------
// Computed in double so the narrowed float is the nearest float to the true
// decimal value (matches float literals in tests exactly).
//
// Temperature: (raw - 2731) / 10 = degC; 0degC -> 2731, 25degC -> 2981,
// -10degC -> 2631.
inline float reg_to_temp_c(uint16_t raw) {
    return static_cast<float>((static_cast<int>(raw) - 2731) * 0.1);
}
// Current: int16 two's complement, charge positive, discharge negative.
// Spec note 3 example: -10A is written as 64535 (1 LSB off; the correct
// encoding is 64536). Two's complement interpretation is used.
inline float reg_to_current_a(uint16_t raw) {
    return static_cast<float>(static_cast<int16_t>(raw) * 0.01);
}
// Pack voltage / capacities: 10mV / 10mAh units.
inline float reg_to_pack_voltage_v(uint16_t raw) {
    return static_cast<float>(raw * 0.01);
}
inline float reg_to_capacity_ah(uint16_t raw) {
    return static_cast<float>(raw * 0.01);
}
// Cell voltage: mV.
inline float reg_to_cell_voltage_v(uint16_t raw) {
    return static_cast<float>(raw * 0.001);
}
// RSOC: high byte reserved, low byte = percent.
inline uint8_t reg_to_rsoc_percent(uint16_t raw) {
    return static_cast<uint8_t>(raw & 0xFFu);
}

// ---- Protection status bits (register 42) ---------------------------------
enum ProtectionBit : uint16_t {
    PROT_NONE = 0,
    PROT_CELL_OVP = 1u << 0,
    PROT_CELL_UVP = 1u << 1,
    PROT_PACK_OVP = 1u << 2,
    PROT_PACK_UVP = 1u << 3,
    PROT_CHG_OTP = 1u << 4,    // charge over-temperature
    PROT_CHG_UTP = 1u << 5,    // charge under-temperature
    PROT_DIS_OTP = 1u << 6,    // discharge over-temperature
    PROT_DIS_UTP = 1u << 7,    // discharge under-temperature
    PROT_CHG_OCP = 1u << 8,    // charge overcurrent
    PROT_DIS_OCP = 1u << 9,    // discharge overcurrent
    PROT_SHORT_CIRCUIT = 1u << 10,
    PROT_AFE_ERROR = 1u << 11,     // front-end detection IC error
    PROT_MOS_SOFT_LOCK = 1u << 12,  // software MOS lock
    PROT_KNOWN_MASK = 0x1FFFu,     // bit13..15 reserved
};

inline const char* protection_bit_name(uint16_t bit) {
    switch (bit) {
        case PROT_CELL_OVP: return "cell overvoltage";
        case PROT_CELL_UVP: return "cell undervoltage";
        case PROT_PACK_OVP: return "pack overvoltage";
        case PROT_PACK_UVP: return "pack undervoltage";
        case PROT_CHG_OTP: return "charge over-temperature";
        case PROT_CHG_UTP: return "charge under-temperature";
        case PROT_DIS_OTP: return "discharge over-temperature";
        case PROT_DIS_UTP: return "discharge under-temperature";
        case PROT_CHG_OCP: return "charge overcurrent";
        case PROT_DIS_OCP: return "discharge overcurrent";
        case PROT_SHORT_CIRCUIT: return "short circuit";
        case PROT_AFE_ERROR: return "AFE IC error";
        case PROT_MOS_SOFT_LOCK: return "MOS software lock";
        default: return "reserved";
    }
}

}  // namespace bms
