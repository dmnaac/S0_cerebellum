#pragma once

#include "drivers/can_bus.hpp"
#include "drivers/power_board_types.hpp"

#include <cstdint>
#include <string>

namespace cerebellum {
namespace drivers {

// RobStride power board driver (protocol V1.5, 20260507).
// CAN 2.0 extended frame, 1 Mbps; frame = 29-bit ID + 8-byte payload.
// All queries are synchronous: send request, wait for the matching reply
// frame (stale/foreign frames are skipped) until timeout.
class PowerBoard {
public:
    PowerBoard(std::string can_iface, uint8_t power_id = 0xAA);
    ~PowerBoard();

    bool open();

    // Type 01: rail switches (0xA1 = on, 0xA0 = off); caches the last state.
    bool setRails(bool v12, bool vmbus, bool v24);
    // Type 01 Byte3: board restart.
    bool restart();
    // Type 01 Byte4: clear latched faults.
    bool clearFault();

    // Type 02: queries. Reply types 03 / 04.
    bool queryStatus(PowerStatus& out, int timeout_ms = 50);
    bool queryCurrents(PowerCurrents& out, int timeout_ms = 50);

    // Type 05: firmware version (ASCII, e.g. "PBV1.00").
    bool queryVersion(std::string& out, int timeout_ms = 50);

    // Type 06: auto report on/off (not used in poll mode).
    bool setAutoReport(bool enable);

    // Type 07 / 08: parameter read/write (index little-endian).
    // Parameter set per protocol doc: 6001~6007, 8001, 8003, 8004.
    bool readParameter(uint16_t index, float& value, int timeout_ms = 50);
    bool writeParameter(uint16_t index, float value);

    // Frame parsers (exposed for the auto-report receive loop).
    // comm type is taken from ID bits 24~28.
    static bool parseStatusFrame(const CanFrame& frame, PowerStatus& out);
    static bool parseCurrentsFrame(const CanFrame& frame, PowerCurrents& out);

    // Raw receive for auto-report mode; returns false on timeout.
    bool receiveRaw(CanFrame& frame, int timeout_ms);

    void getRailState(bool& v12, bool& vmbus, bool& v24) const;

private:
    static constexpr uint8_t COMM_CONTROL     = 0x01;
    static constexpr uint8_t COMM_QUERY       = 0x02;
    static constexpr uint8_t COMM_STATUS      = 0x03;
    static constexpr uint8_t COMM_CURRENTS    = 0x04;
    static constexpr uint8_t COMM_VERSION     = 0x05;
    static constexpr uint8_t COMM_AUTO_REPORT = 0x06;
    static constexpr uint8_t COMM_READ_PARAM  = 0x07;
    static constexpr uint8_t COMM_WRITE_PARAM = 0x08;

    bool sendFrame(uint8_t comm_type, const uint8_t* data);
    void drain(int timeout_ms);

    CanBus bus_;
    std::string iface_;
    uint8_t power_id_;
    bool v12_ = false;
    bool vmbus_ = false;
    bool v24_ = false;
};

} // namespace drivers
} // namespace cerebellum
