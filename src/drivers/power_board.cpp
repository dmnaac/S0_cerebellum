#include "drivers/power_board.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
#include <utility>

namespace cerebellum {
namespace drivers {

namespace {

// Little-endian float <-> 4-byte array helpers for parameter frames
// (protocol: parameter index and value are both low-byte first).
float leBytesToFloat(const uint8_t* d) {
    uint32_t raw = static_cast<uint32_t>(d[0]) |
                   (static_cast<uint32_t>(d[1]) << 8) |
                   (static_cast<uint32_t>(d[2]) << 16) |
                   (static_cast<uint32_t>(d[3]) << 24);
    float value;
    std::memcpy(&value, &raw, sizeof(float));
    return value;
}

void floatToLeBytes(float value, uint8_t* d) {
    uint32_t raw;
    std::memcpy(&raw, &value, sizeof(float));
    d[0] = static_cast<uint8_t>(raw & 0xFF);
    d[1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    d[2] = static_cast<uint8_t>((raw >> 16) & 0xFF);
    d[3] = static_cast<uint8_t>((raw >> 24) & 0xFF);
}

uint8_t commTypeOf(const CanFrame& frame) {
    return static_cast<uint8_t>((frame.id >> 24) & 0x1F);
}

} // namespace

PowerBoard::PowerBoard(std::string can_iface, uint8_t power_id)
    : iface_(std::move(can_iface)), power_id_(power_id) {}

PowerBoard::~PowerBoard() = default;

bool PowerBoard::open() {
    return bus_.open(iface_);
}

bool PowerBoard::sendFrame(uint8_t comm_type, const uint8_t* data) {
    CanFrame frame;
    frame.id = (static_cast<uint32_t>(comm_type) << 24) |
               static_cast<uint32_t>(power_id_);
    frame.extended = true;
    frame.dlc = 8;
    if (data) {
        std::memcpy(frame.data.data(), data, 8);
    } else {
        frame.data.fill(0);
    }
    if (!bus_.send(frame)) {
        std::cerr << "power_board: failed to send frame, comm_type=0x" << std::hex
                  << static_cast<int>(comm_type) << std::dec << std::endl;
        return false;
    }
    return true;
}

void PowerBoard::drain(int timeout_ms) {
    CanFrame frame;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        int remain = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline -
                                                                  std::chrono::steady_clock::now())
                .count());
        if (remain <= 0 || !bus_.receive(frame, remain)) {
            break;
        }
    }
}

bool PowerBoard::setRails(bool v12, bool vmbus, bool v24) {
    uint8_t data[8] = {0};
    data[0] = v12 ? 0xA1 : 0xA0;   // 12V
    data[1] = vmbus ? 0xA1 : 0xA0; // VMBUS
    data[2] = v24 ? 0xA1 : 0xA0;   // 24V
    if (!sendFrame(COMM_CONTROL, data)) {
        return false;
    }
    v12_ = v12;
    vmbus_ = vmbus;
    v24_ = v24;
    drain(10); // reply is a type 03 frame; not consumed here
    return true;
}

bool PowerBoard::restart() {
    uint8_t data[8] = {0};
    data[0] = v12_ ? 0xA1 : 0xA0;
    data[1] = vmbus_ ? 0xA1 : 0xA0;
    data[2] = v24_ ? 0xA1 : 0xA0;
    data[3] = 0xA1; // restart
    if (!sendFrame(COMM_CONTROL, data)) {
        return false;
    }
    drain(10);
    return true;
}

bool PowerBoard::clearFault() {
    uint8_t data[8] = {0};
    data[0] = v12_ ? 0xA1 : 0xA0;
    data[1] = vmbus_ ? 0xA1 : 0xA0;
    data[2] = v24_ ? 0xA1 : 0xA0;
    data[4] = 0xA1; // clear fault
    if (!sendFrame(COMM_CONTROL, data)) {
        return false;
    }
    drain(10);
    return true;
}

bool PowerBoard::queryStatus(PowerStatus& out, int timeout_ms) {
    uint8_t data[8] = {0};
    data[0] = 0x00; // request type 03 reply
    if (!sendFrame(COMM_QUERY, data)) {
        return false;
    }

    CanFrame frame;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        int remain = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline -
                                                                  std::chrono::steady_clock::now())
                .count());
        if (remain <= 0 || !bus_.receive(frame, remain)) {
            break;
        }
        if (parseStatusFrame(frame, out)) {
            return true;
        }
        // stale/foreign frame: keep waiting
    }
    return false;
}

bool PowerBoard::queryCurrents(PowerCurrents& out, int timeout_ms) {
    uint8_t data[8] = {0};
    data[0] = 0xCA; // request type 04 reply
    if (!sendFrame(COMM_QUERY, data)) {
        return false;
    }

    CanFrame frame;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        int remain = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline -
                                                                  std::chrono::steady_clock::now())
                .count());
        if (remain <= 0 || !bus_.receive(frame, remain)) {
            break;
        }
        if (parseCurrentsFrame(frame, out)) {
            return true;
        }
    }
    return false;
}

bool PowerBoard::queryVersion(std::string& out, int timeout_ms) {
    uint8_t data[8] = {0};
    if (!sendFrame(COMM_VERSION, data)) {
        return false;
    }

    CanFrame frame;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        int remain = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline -
                                                                  std::chrono::steady_clock::now())
                .count());
        if (remain <= 0 || !bus_.receive(frame, remain)) {
            break;
        }
        if (commTypeOf(frame) != COMM_VERSION) {
            continue;
        }
        out.clear();
        for (uint8_t b : frame.data) {
            if (b == 0) break;
            out.push_back(static_cast<char>(b));
        }
        return true;
    }
    return false;
}

bool PowerBoard::setAutoReport(bool enable) {
    uint8_t data[8] = {0};
    data[0] = enable ? 0x01 : 0x00;
    if (!sendFrame(COMM_AUTO_REPORT, data)) {
        return false;
    }
    drain(10);
    return true;
}

bool PowerBoard::readParameter(uint16_t index, float& value, int timeout_ms) {
    uint8_t data[8] = {0};
    data[0] = static_cast<uint8_t>(index & 0xFF);
    data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
    if (!sendFrame(COMM_READ_PARAM, data)) {
        return false;
    }

    CanFrame frame;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        int remain = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline -
                                                                  std::chrono::steady_clock::now())
                .count());
        if (remain <= 0 || !bus_.receive(frame, remain)) {
            break;
        }
        if (commTypeOf(frame) != COMM_READ_PARAM) {
            continue;
        }
        uint16_t reply_index = static_cast<uint16_t>(frame.data[0]) |
                               (static_cast<uint16_t>(frame.data[1]) << 8);
        if (reply_index != index) {
            continue; // mismatched reply, keep waiting
        }
        value = leBytesToFloat(&frame.data[4]);
        return true;
    }
    return false;
}

bool PowerBoard::writeParameter(uint16_t index, float value) {
    uint8_t data[8] = {0};
    data[0] = static_cast<uint8_t>(index & 0xFF);
    data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
    floatToLeBytes(value, &data[4]);
    if (!sendFrame(COMM_WRITE_PARAM, data)) {
        return false;
    }
    drain(10); // reply is a type 03 frame
    return true;
}

bool PowerBoard::receiveRaw(CanFrame& frame, int timeout_ms) {
    return bus_.receive(frame, timeout_ms);
}

bool PowerBoard::parseStatusFrame(const CanFrame& frame, PowerStatus& out) {
    if (commTypeOf(frame) != COMM_STATUS) {
        return false;
    }

    // Current sampling: doc lists it in ID bits 16~23 with a 0~65535 range,
    // which requires 16 bits; parsed from ID bits 8~23 (0.01 A per LSB).
    // NOTE: verify against the real board; this is the doc-ambiguous field.
    out.bus_current = static_cast<float>((frame.id >> 8) & 0xFFFF) * 0.01f;

    // Voltages: 0.01 V per LSB, big-endian (doc does not state byte order;
    // matches the RobStride motion frames — verify on board if readings look wrong).
    out.battery_voltage =
        static_cast<float>((static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1]) *
        0.01f;
    out.motor_voltage =
        static_cast<float>((static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3]) *
        0.01f;
    out.temperature = static_cast<int>(frame.data[4]);

    // Fault + rail status field: 24 bits, Byte5 is the most significant byte
    // (doc places rail states at bit20~23, i.e. within Byte5).
    out.fault_bits = (static_cast<uint32_t>(frame.data[5]) << 16) |
                     (static_cast<uint32_t>(frame.data[6]) << 8) |
                     static_cast<uint32_t>(frame.data[7]);
    out.rail12v_on = (out.fault_bits & POWER_STATUS_12V_ON) != 0;
    out.softstart_on = (out.fault_bits & POWER_STATUS_SOFTSTART_ON) != 0;
    out.vmbus_on = (out.fault_bits & POWER_STATUS_VMBUS_ON) != 0;
    out.rail24v_on = (out.fault_bits & POWER_STATUS_24V_ON) != 0;
    return true;
}

bool PowerBoard::parseCurrentsFrame(const CanFrame& frame, PowerCurrents& out) {
    if (commTypeOf(frame) != COMM_CURRENTS) {
        return false;
    }
    auto u16be = [](uint8_t hi, uint8_t lo) {
        return static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8) | lo);
    };
    out.al = static_cast<float>(u16be(frame.data[0], frame.data[1])) * 0.01f;
    out.ar = static_cast<float>(u16be(frame.data[2], frame.data[3])) * 0.01f;
    out.ll = static_cast<float>(u16be(frame.data[4], frame.data[5])) * 0.01f;
    out.lr = static_cast<float>(u16be(frame.data[6], frame.data[7])) * 0.01f;
    return true;
}

void PowerBoard::getRailState(bool& v12, bool& vmbus, bool& v24) const {
    v12 = v12_;
    vmbus = vmbus_;
    v24 = v24_;
}

} // namespace drivers
} // namespace cerebellum
