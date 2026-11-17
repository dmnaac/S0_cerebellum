// Error codes and data model.
#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "bms/registers.hpp"

namespace bms {

// Transaction error codes (no exceptions on the data path).
enum class Error : uint8_t {
    Ok = 0,
    Timeout,            // no or incomplete response within the timeout
    CrcMismatch,        // response CRC check failed
    FrameMalformed,     // wrong slave address / function / byte count / length
    ModbusException,    // slave replied with an exception code
    IoError,            // transport open/write/read failure
    RetryExhausted,     // all attempts failed; see last_error() for the cause
    InvalidArgument,
};

inline const char* error_name(Error e) {
    switch (e) {
        case Error::Ok: return "ok";
        case Error::Timeout: return "timeout";
        case Error::CrcMismatch: return "crc mismatch";
        case Error::FrameMalformed: return "frame malformed";
        case Error::ModbusException: return "modbus exception";
        case Error::IoError: return "io error";
        case Error::RetryExhausted: return "retry exhausted";
        case Error::InvalidArgument: return "invalid argument";
    }
    return "unknown";
}

// Standard Modbus exception codes (subset this BMS may return).
enum class ModbusExceptionCode : uint8_t {
    IllegalFunction = 0x01,
    IllegalDataAddress = 0x02,
    IllegalDataValue = 0x03,
    SlaveDeviceFailure = 0x04,
    SlaveBusy = 0x06,
    MemoryParityError = 0x08,
};

inline const char* modbus_exception_name(ModbusExceptionCode c) {
    switch (c) {
        case ModbusExceptionCode::IllegalFunction: return "illegal function";
        case ModbusExceptionCode::IllegalDataAddress: return "illegal data address";
        case ModbusExceptionCode::IllegalDataValue: return "illegal data value";
        case ModbusExceptionCode::SlaveDeviceFailure: return "slave device failure";
        case ModbusExceptionCode::SlaveBusy: return "slave busy";
        case ModbusExceptionCode::MemoryParityError: return "memory parity error";
    }
    return "unknown";
}

// Protection parameters (registers 0~17), raw values with unit suffixes.
struct ProtectionParams {
    uint16_t cell_ovp_mV = 0;
    uint16_t cell_ovp_release_mV = 0;
    uint16_t cell_uvp_mV = 0;
    uint16_t cell_uvp_release_mV = 0;
    uint16_t pack_ovp_10mV = 0;
    uint16_t pack_ovp_release_10mV = 0;
    uint16_t pack_uvp_10mV = 0;
    uint16_t pack_uvp_release_10mV = 0;
    uint16_t chg_high_temp_01K = 0;
    uint16_t chg_high_temp_release_01K = 0;
    uint16_t chg_low_temp_01K = 0;
    uint16_t chg_low_temp_release_01K = 0;
    uint16_t dis_high_temp_01K = 0;
    uint16_t dis_high_temp_release_01K = 0;
    uint16_t dis_low_temp_01K = 0;
    uint16_t dis_low_temp_release_01K = 0;
    uint16_t chg_ocp_10mA = 0;
    uint16_t dis_ocp_10mA = 0;
};

// Live status (registers 18~42), converted to engineering units.
struct BatteryStatus {
    uint32_t timestamp_ms = 0;                       // steady clock ms at sampling
    float pack_voltage_V = 0.0f;                     // reg 18
    float current_A = 0.0f;                          // reg 19, charge positive
    float remaining_capacity_Ah = 0.0f;              // reg 20
    float nominal_capacity_Ah = 0.0f;                // reg 39
    std::array<float, kCellCount> cell_voltages_V{};  // regs 21..36
    std::array<float, 2> temps_C{};                  // regs 37, 38
    uint8_t rsoc_percent = 0;                        // reg 40 low byte
    uint16_t fet_raw = 0;                            // reg 41
    uint16_t protect_status = 0;                     // reg 42
    bool mos_charge_closed = false;
    bool mos_discharge_closed = false;

    bool protection_active() const { return protect_status != 0; }

    float cell_voltage_min_V() const {
        float m = cell_voltages_V[0];
        for (float v : cell_voltages_V) {
            if (v < m) {
                m = v;
            }
        }
        return m;
    }

    float cell_voltage_max_V() const {
        float m = cell_voltages_V[0];
        for (float v : cell_voltages_V) {
            if (v > m) {
                m = v;
            }
        }
        return m;
    }

    float cell_voltage_diff_mV() const {
        return (cell_voltage_max_V() - cell_voltage_min_V()) * 1000.0f;
    }

    std::vector<uint16_t> active_protection_bits() const {
        std::vector<uint16_t> bits;
        for (uint16_t bit = 1; bit <= PROT_MOS_SOFT_LOCK;
             bit = static_cast<uint16_t>(bit << 1)) {
            if ((protect_status & bit) != 0) {
                bits.push_back(bit);
            }
        }
        return bits;
    }
};

}  // namespace bms
