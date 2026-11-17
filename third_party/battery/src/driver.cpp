#include "bms/driver.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <stdexcept>
#include <utility>

#include "bms/frame.hpp"
#include "bms/registers.hpp"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <time.h>
#endif

namespace bms {
namespace {

constexpr std::size_t kRequestLen = 8;

void sleep_ms(int ms) {
    if (ms <= 0) {
        return;
    }
#ifdef _WIN32
    ::Sleep(static_cast<unsigned long>(ms));
#else
    struct timespec ts{};
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = static_cast<long>(ms % 1000) * 1000000L;
    ::nanosleep(&ts, nullptr);
#endif
}

uint32_t now_ms() {
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

}  // namespace

BmsDriver::BmsDriver(std::unique_ptr<ITransport> transport, uint8_t slave)
    : transport_(std::move(transport)), slave_(slave) {
    if (!transport_) {
        throw std::invalid_argument("transport must not be null");
    }
}

void BmsDriver::log_frame(const char* tag, const uint8_t* data,
                          std::size_t len) const {
    if (!verbose_) {
        return;
    }
    std::fprintf(stderr, "[bms] %s", tag);
    if (data != nullptr && len > 0) {
        std::fprintf(stderr, " (%zu bytes):", len);
        for (std::size_t i = 0; i < len; ++i) {
            std::fprintf(stderr, " %02X", data[i]);
        }
    }
    std::fprintf(stderr, "\n");
    std::fflush(stderr);
}

// One transaction: build request, write, read response incrementally,
// parse. Retries on timeout/CRC/frame errors; returns immediately on a
// slave exception (the slave answered).
Error BmsDriver::transaction_read(uint16_t start, uint16_t count,
                                  uint16_t* out) {
    if (count == 0 || count > 125) {
        return Error::InvalidArgument;
    }

    std::array<uint8_t, kRequestLen> req{};
    const std::size_t req_len = build_read_request(
        slave_, static_cast<uint16_t>(start + kAddressOffset), count,
        req.data());

    Error attempt_error = Error::Ok;
    for (int attempt = 0; attempt <= kRetries; ++attempt) {
        if (attempt > 0) {
            sleep_ms(kInterFrameDelayMs);
        }
        attempt_error = Error::Ok;

        if (!transport_->is_open()) {
            attempt_error = Error::IoError;
            continue;
        }
        if (!transport_->write(req.data(), req_len)) {
            attempt_error = Error::IoError;
            continue;
        }
        log_frame("TX", req.data(), req_len);

        // Header first: slave + function.
        uint8_t head[2] = {0, 0};
        if (!transport_->read_exact(head, 2, kTimeoutMs)) {
            attempt_error = Error::Timeout;
            log_frame("RX timeout (header)", nullptr, 0);
            continue;
        }
        if (head[0] != slave_) {
            attempt_error = Error::FrameMalformed;
            continue;
        }

        const ResponseHead kind = classify_response_head(head[1]);
        if (kind == ResponseHead::Exception) {
            uint8_t rest[3] = {0, 0, 0};
            if (!transport_->read_exact(rest, 3, kTimeoutMs)) {
                attempt_error = Error::Timeout;
                continue;
            }
            uint8_t frame[kExceptionResponseLen];
            frame[0] = head[0];
            frame[1] = head[1];
            frame[2] = rest[0];
            frame[3] = rest[1];
            frame[4] = rest[2];
            log_frame("RX", frame, kExceptionResponseLen);
            ModbusExceptionCode code{};
            const Error e = parse_exception_response(
                frame, kExceptionResponseLen, slave_, &code);
            if (e == Error::ModbusException) {
                last_exception_ = code;
                last_error_ = e;
                return e;
            }
            attempt_error = e;
            continue;
        }
        if (kind != ResponseHead::Read) {
            attempt_error = Error::FrameMalformed;
            continue;
        }

        // Byte count, then data + CRC.
        uint8_t byte_count = 0;
        if (!transport_->read_exact(&byte_count, 1, kTimeoutMs)) {
            attempt_error = Error::Timeout;
            continue;
        }
        if (byte_count != static_cast<uint8_t>(2 * count)) {
            attempt_error = Error::FrameMalformed;
            continue;
        }
        const std::size_t body = 2 * static_cast<std::size_t>(count);
        std::array<uint8_t, 2 * 125 + 2> buf{};
        if (!transport_->read_exact(buf.data(), body + 2, kTimeoutMs)) {
            attempt_error = Error::Timeout;
            continue;
        }

        std::array<uint8_t, kMaxFrameLen> frame{};
        frame[0] = head[0];
        frame[1] = head[1];
        frame[2] = byte_count;
        for (std::size_t i = 0; i < body + 2; ++i) {
            frame[3 + i] = buf[i];
        }
        const std::size_t frame_len = 3 + body + 2;
        log_frame("RX", frame.data(), frame_len);

        const Error e =
            parse_read_response(frame.data(), frame_len, slave_, count, out);
        if (e == Error::Ok) {
            last_error_ = Error::Ok;
            return Error::Ok;
        }
        attempt_error = e;
    }

    last_error_ = attempt_error;
    return Error::RetryExhausted;
}

Error BmsDriver::read_registers(uint16_t start, uint16_t count,
                                uint16_t* out) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (out == nullptr || count == 0) {
        return Error::InvalidArgument;
    }
    if (static_cast<std::size_t>(start) + count >
        static_cast<std::size_t>(REG_LAST) + 1) {
        return Error::InvalidArgument;
    }

    std::size_t done = 0;
    while (done < count) {
        const std::size_t remaining = count - done;
        const std::size_t n =
            remaining < static_cast<std::size_t>(kChunkSize)
                ? remaining
                : static_cast<std::size_t>(kChunkSize);
        sleep_ms(kInterFrameDelayMs);
        const Error e = transaction_read(
            static_cast<uint16_t>(start + done), static_cast<uint16_t>(n),
            out + done);
        if (e != Error::Ok) {
            return e;
        }
        done += n;
    }
    return Error::Ok;
}

Error BmsDriver::read_parameters(ProtectionParams* out) {
    if (out == nullptr) {
        return Error::InvalidArgument;
    }
    uint16_t raw[REG_PARAM_COUNT] = {0};
    const Error e = read_registers(REG_PARAM_FIRST, REG_PARAM_COUNT, raw);
    if (e != Error::Ok) {
        return e;
    }
    out->cell_ovp_mV = raw[0];
    out->cell_ovp_release_mV = raw[1];
    out->cell_uvp_mV = raw[2];
    out->cell_uvp_release_mV = raw[3];
    out->pack_ovp_10mV = raw[4];
    out->pack_ovp_release_10mV = raw[5];
    out->pack_uvp_10mV = raw[6];
    out->pack_uvp_release_10mV = raw[7];
    out->chg_high_temp_01K = raw[8];
    out->chg_high_temp_release_01K = raw[9];
    out->chg_low_temp_01K = raw[10];
    out->chg_low_temp_release_01K = raw[11];
    out->dis_high_temp_01K = raw[12];
    out->dis_high_temp_release_01K = raw[13];
    out->dis_low_temp_01K = raw[14];
    out->dis_low_temp_release_01K = raw[15];
    out->chg_ocp_10mA = raw[16];
    out->dis_ocp_10mA = raw[17];
    return Error::Ok;
}

Error BmsDriver::read_status(BatteryStatus* out) {
    if (out == nullptr) {
        return Error::InvalidArgument;
    }
    constexpr uint16_t kCount = REG_LAST - REG_PACK_VOLTAGE + 1;  // 25
    uint16_t raw[kCount] = {0};
    const Error e = read_registers(REG_PACK_VOLTAGE, kCount, raw);
    if (e != Error::Ok) {
        return e;
    }

    out->pack_voltage_V = reg_to_pack_voltage_v(raw[0]);
    out->current_A = reg_to_current_a(raw[1]);
    out->remaining_capacity_Ah = reg_to_capacity_ah(raw[2]);
    for (std::size_t i = 0; i < kCellCount; ++i) {
        out->cell_voltages_V[i] = reg_to_cell_voltage_v(raw[3 + i]);
    }
    out->temps_C[0] = reg_to_temp_c(raw[3 + kCellCount]);       // reg 37
    out->temps_C[1] = reg_to_temp_c(raw[3 + kCellCount + 1]);   // reg 38
    out->nominal_capacity_Ah =
        reg_to_capacity_ah(raw[3 + kCellCount + 2]);            // reg 39
    out->rsoc_percent = reg_to_rsoc_percent(raw[3 + kCellCount + 3]);  // reg 40
    out->fet_raw = raw[3 + kCellCount + 4];                     // reg 41
    // FET read semantics: the spec only defines write semantics (1 = close,
    // 0 = open). The read encoding is assumed identical; verify on hardware
    // (if inverted, flip the two lines below only).
    out->mos_charge_closed = (out->fet_raw & 0x01u) != 0;
    out->mos_discharge_closed = (out->fet_raw & 0x02u) != 0;
    out->protect_status = raw[3 + kCellCount + 5];              // reg 42
    out->timestamp_ms = now_ms();
    return Error::Ok;
}

Error BmsDriver::read_all(BatteryStatus* status, ProtectionParams* params) {
    if (status == nullptr || params == nullptr) {
        return Error::InvalidArgument;
    }
    const Error e1 = read_parameters(params);
    if (e1 != Error::Ok) {
        return e1;
    }
    const Error e2 = read_status(status);
    if (e2 != Error::Ok) {
        return e2;
    }
    return Error::Ok;
}

}  // namespace bms
