#pragma once

#include <cstdint>

namespace cerebellum {
namespace drivers {

// Power board fault bits (protocol doc, type 03 frame, Byte5~Byte7, 24-bit field)
enum PowerFaultBit {
    POWER_FAULT_CHIP_OVER_CURRENT   = 0,   // 电源芯片过流
    POWER_FAULT_CHIP_OVER_TEMP      = 1,   // 电源芯片过温
    POWER_FAULT_BOARD_OVER_TEMP     = 2,   // 电源板过温 (100 degC)
    POWER_FAULT_SAMPLE_OVER_CURRENT = 3,   // 采样过流
    POWER_FAULT_VBUS_OVER_VOLT      = 4,   // 采样电压 vbus 过压
    POWER_FAULT_VBUS_UNDER_VOLT     = 5,   // 采样电压 vbus 欠压
    POWER_FAULT_VMBUS_OVER_VOLT     = 6,   // 采样电压 vmbus 过压
    POWER_FAULT_VMBUS_UNDER_VOLT    = 7,   // 采样电压 vmbus 欠压
    POWER_FAULT_24V_OVER_VOLT       = 8,   // 采样电压 24V 过压
    POWER_FAULT_24V_UNDER_VOLT      = 9,   // 采样电压 24V 欠压
    POWER_FAULT_12V_OVER_VOLT       = 10,  // 采样电压 12V 过压
    POWER_FAULT_12V_UNDER_VOLT      = 11,  // 采样电压 12V 欠压
    POWER_FAULT_BRANCH_OVER_CURRENT = 12,  // 输出支路采样过流
    POWER_FAULT_INA238              = 13,  // ina238 故障帧
    POWER_FAULT_PRECHARGE           = 14,  // 缓充异常故障帧
};

// Rail status bits inside the same 24-bit field (bit20~23)
constexpr uint32_t POWER_STATUS_12V_ON       = 1u << 20;
constexpr uint32_t POWER_STATUS_SOFTSTART_ON = 1u << 21;
constexpr uint32_t POWER_STATUS_VMBUS_ON     = 1u << 22;
constexpr uint32_t POWER_STATUS_24V_ON       = 1u << 23;

constexpr uint32_t POWER_FAULT_MASK = 0x7FFF;  // bit0~14

struct PowerStatus {
    float battery_voltage;  // V
    float motor_voltage;    // V (VMBUS)
    float bus_current;      // A
    int temperature;        // deg C
    uint32_t fault_bits;    // fault bits (0~14) + rail status bits (20~23)
    bool rail12v_on;
    bool softstart_on;
    bool vmbus_on;
    bool rail24v_on;
};

struct PowerCurrents {
    float al;  // A
    float ar;  // A
    float ll;  // A
    float lr;  // A
};

} // namespace drivers
} // namespace cerebellum
