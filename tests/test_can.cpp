#include "drivers/can_bus.hpp"

#include <iostream>

int main(int argc, char** argv) {
    std::string iface = "can0";
    if (argc > 1) {
        iface = argv[1];
    }

    cerebellum::drivers::CanBus can(iface);
    if (!can.open()) {
        std::cerr << "Failed to open " << iface << std::endl;
        return -1;
    }

    std::cout << "CAN interface " << iface << " opened successfully" << std::endl;

    cerebellum::drivers::CanFrame frame;
    frame.id = 0x123;
    frame.extended = false;
    frame.dlc = 8;
    for (int i = 0; i < 8; ++i) {
        frame.data[i] = static_cast<uint8_t>(i);
    }

    if (can.send(frame)) {
        std::cout << "Sent test frame" << std::endl;
    } else {
        std::cerr << "Failed to send test frame" << std::endl;
    }

    can.close();
    return 0;
}
