// POSIX serial transport (RK3588 / Ubuntu): 9600 8N1 raw UART with
// poll-based exact-length reads, plus two RS485 direction-control modes.
#include "bms/transport.hpp"

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

// RS485 ioctl definitions. The kernel uapi layout (32 bytes) is reproduced
// locally (with a private struct name to avoid clashing with toolchain
// headers) so the ns-precision RTS delays are always available:
//   flags, delay_rts_before_send_ms, delay_rts_after_send_ms,
//   delay_rts_before_send_ns, delay_rts_after_send_ns, padding[3]
#ifndef TIOCSRS485
#define TIOCSRS485 0x542F
#endif
#ifndef SER_RS485_ENABLED
#define SER_RS485_ENABLED (static_cast<uint32_t>(1) << 0)
#endif
#ifndef SER_RS485_RTS_ON_SEND
#define SER_RS485_RTS_ON_SEND (static_cast<uint32_t>(1) << 1)
#endif

namespace bms {
namespace {

constexpr int kWriteTimeoutMs = 1000;

struct bms_serial_rs485 {
    uint32_t flags;
    uint32_t delay_rts_before_send_ms;
    uint32_t delay_rts_after_send_ms;
    uint32_t delay_rts_before_send_ns;
    uint32_t delay_rts_after_send_ns;
    uint32_t padding[3];
};

int baud_to_bflag(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: return -1;
    }
}

enum class Direction { Auto, KernelRs485 };

struct PosixSerialConfig {
    Direction direction = Direction::Auto;
    uint32_t rts_before_send_us = 0;  // kernel RS485: RTS assert lead time
    uint32_t rts_after_send_us = 0;   // kernel RS485: RTS hold after last byte
};

class PosixSerial : public ITransport {
public:
    PosixSerial(const char* device, int baud, PosixSerialConfig config)
        : device_(device), baud_(baud), config_(config) {}
    ~PosixSerial() override { close(); }

    bool open() override;
    void close() override;
    bool is_open() const override { return fd_ >= 0; }
    bool write(const uint8_t* data, std::size_t len) override;
    bool read_exact(uint8_t* data, std::size_t len, int timeout_ms) override;

private:
    bool configure_termios();
    bool configure_rs485();
    bool wait_writable(int timeout_ms);

    const char* device_;
    int baud_;
    PosixSerialConfig config_;
    int fd_ = -1;
};

bool PosixSerial::open() {
    fd_ = ::open(device_, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::fprintf(stderr, "[bms] open(%s) failed: %s\n", device_,
                     std::strerror(errno));
        return false;
    }
    if (!configure_termios()) {
        close();
        return false;
    }
    if (config_.direction == Direction::KernelRs485 && !configure_rs485()) {
        // Kernel RS485 mode unavailable (UART/driver may not support it, or
        // DE is not wired to RTS). Continue: the module may be auto-direction.
        std::fprintf(stderr,
                     "[bms] warning: TIOCSRS485 on %s failed: %s; continuing "
                     "without kernel RS485 mode\n",
                     device_, std::strerror(errno));
    }
    return true;
}

void PosixSerial::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool PosixSerial::configure_termios() {
    struct termios tio{};
    if (::tcgetattr(fd_, &tio) != 0) {
        std::fprintf(stderr, "[bms] tcgetattr(%s) failed: %s\n", device_,
                     std::strerror(errno));
        return false;
    }
    const int bflag = baud_to_bflag(baud_);
    if (bflag < 0) {
        std::fprintf(stderr, "[bms] unsupported baud rate %d\n", baud_);
        return false;
    }
    ::cfmakeraw(&tio);
    // 8 data bits, no parity, 1 stop bit (8N1), no hardware flow control.
    tio.c_cflag = static_cast<tcflag_t>(bflag) | CS8 | CLOCAL | CREAD;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
        std::fprintf(stderr, "[bms] tcsetattr(%s) failed: %s\n", device_,
                     std::strerror(errno));
        return false;
    }
    ::tcflush(fd_, TCIOFLUSH);
    return true;
}

bool PosixSerial::configure_rs485() {
    bms_serial_rs485 cfg{};
    cfg.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND;
    cfg.delay_rts_before_send_ms = config_.rts_before_send_us / 1000u;
    cfg.delay_rts_after_send_ms = config_.rts_after_send_us / 1000u;
    // ns precision takes precedence in the kernel when non-zero; needed for
    // sub-millisecond first-byte-loss tuning at 9600bps (1 byte ~1.04ms).
    cfg.delay_rts_before_send_ns = config_.rts_before_send_us * 1000u;
    cfg.delay_rts_after_send_ns = config_.rts_after_send_us * 1000u;
    return ::ioctl(fd_, TIOCSRS485, &cfg) == 0;
}

bool PosixSerial::wait_writable(int timeout_ms) {
    struct pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLOUT;
    return ::poll(&pfd, 1, timeout_ms) > 0;
}

bool PosixSerial::write(const uint8_t* data, std::size_t len) {
    if (fd_ < 0 || data == nullptr || len == 0) {
        return false;
    }
    // Drop stale bytes from a previous timed-out attempt so the retry does
    // not desync on a late response.
    ::tcflush(fd_, TCIFLUSH);
    std::size_t sent = 0;
    while (sent < len) {
        const ssize_t n = ::write(fd_, data + sent, len - sent);
        if (n > 0) {
            sent += static_cast<std::size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            if (!wait_writable(kWriteTimeoutMs)) {
                return false;
            }
            continue;
        }
        return false;
    }
    return true;
}

bool PosixSerial::read_exact(uint8_t* data, std::size_t len, int timeout_ms) {
    if (fd_ < 0 || data == nullptr || len == 0) {
        return false;
    }
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    std::size_t got = 0;
    while (got < len) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            return false;
        }
        const auto remaining =
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)
                .count();
        struct pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN;
        const int r = ::poll(&pfd, 1, static_cast<int>(remaining));
        if (r < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        if (r == 0) {
            return false;  // timeout
        }
        const ssize_t n = ::read(fd_, data + got, len - got);
        if (n > 0) {
            got += static_cast<std::size_t>(n);
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            continue;
        }
        return false;
    }
    return true;
}

}  // namespace

std::unique_ptr<ITransport> make_posix_serial(const char* device, int baud,
                                              const char* direction,
                                              uint32_t rts_before_send_us,
                                              uint32_t rts_after_send_us) {
    PosixSerialConfig cfg;
    if (direction != nullptr && std::strcmp(direction, "kernel") == 0) {
        cfg.direction = Direction::KernelRs485;
    } else {
        cfg.direction = Direction::Auto;
    }
    cfg.rts_before_send_us = rts_before_send_us;
    cfg.rts_after_send_us = rts_after_send_us;
    return std::make_unique<PosixSerial>(device, baud, cfg);
}

}  // namespace bms
