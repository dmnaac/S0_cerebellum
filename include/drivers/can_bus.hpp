#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace cerebellum {
namespace drivers {

struct CanFrame {
    uint32_t id;
    bool extended;
    std::array<uint8_t, 8> data;
    uint8_t dlc;
};

class CanBus {
public:
    explicit CanBus(const std::string& interface);
    ~CanBus();

    bool open();
    void close();
    bool isOpen() const;

    bool send(const CanFrame& frame);
    bool receive(CanFrame& frame, int timeout_ms = 0);

private:
    std::string interface_;
    int socket_fd_;
};

} // namespace drivers
} // namespace cerebellum
