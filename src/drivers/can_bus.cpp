#include "drivers/can_bus.hpp"

#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace cerebellum {
namespace drivers {

CanBus::CanBus(const std::string& interface) : interface_(interface), socket_fd_(-1) {}

CanBus::~CanBus() {
    close();
}

bool CanBus::open() {
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        std::cerr << "Failed to create CAN socket" << std::endl;
        return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Failed to get interface index for " << interface_ << std::endl;
        close();
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "Failed to bind CAN socket" << std::endl;
        close();
        return false;
    }

    return true;
}

void CanBus::close() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool CanBus::isOpen() const {
    return socket_fd_ >= 0;
}

bool CanBus::send(const CanFrame& frame) {
    if (!isOpen()) return false;

    struct canfd_frame raw;
    // Use classic CAN frame
    struct can_frame cf;
    cf.can_id = frame.id | (frame.extended ? CAN_EFF_FLAG : 0);
    cf.can_dlc = frame.dlc;
    std::memcpy(cf.data, frame.data.data(), 8);

    return ::write(socket_fd_, &cf, sizeof(cf)) == sizeof(cf);
}

bool CanBus::receive(CanFrame& frame, int timeout_ms) {
    if (!isOpen()) return false;

    struct pollfd pfd;
    pfd.fd = socket_fd_;
    pfd.events = POLLIN;

    int ret = poll(&pfd, 1, timeout_ms);
    if (ret <= 0) return false;

    struct can_frame cf;
    ssize_t n = ::read(socket_fd_, &cf, sizeof(cf));
    if (n != sizeof(cf)) return false;

    frame.id = cf.can_id & CAN_EFF_MASK;
    frame.extended = (cf.can_id & CAN_EFF_FLAG) != 0;
    frame.dlc = cf.can_dlc;
    std::memcpy(frame.data.data(), cf.data, 8);
    return true;
}

} // namespace drivers
} // namespace cerebellum
