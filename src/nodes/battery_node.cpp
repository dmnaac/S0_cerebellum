#include <lcm/lcm-cpp.hpp>
#include <quad/battery_status_t.hpp>

#include "bms/driver.hpp"
#include "bms/transport.hpp"
#include "bms/types.hpp"

#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace {

volatile std::sig_atomic_t g_stop = 0;

void onSignal(int) {
    g_stop = 1;
}

struct BatteryNodeConfig {
    std::string port = "/dev/ttyS1";
    int baud = 9600;               // fixed by the Modbus spec
    int slave = 1;
    std::string rs485_direction = "auto";  // "auto" | "kernel"
    uint32_t rts_before_us = 0;
    uint32_t rts_after_us = 0;
    int poll_interval_ms = 1000;
    int param_read_interval_s = 60;
    int max_consecutive_errors = 5;
};

bool loadConfig(const std::string& filepath, BatteryNodeConfig& cfg) {
    try {
        YAML::Node root = YAML::LoadFile(filepath);
        cfg.port = root["port"].as<std::string>(cfg.port);
        cfg.baud = root["baud"].as<int>(cfg.baud);
        cfg.slave = root["slave"].as<int>(cfg.slave);
        cfg.rs485_direction = root["rs485_direction"].as<std::string>(cfg.rs485_direction);
        cfg.rts_before_us = root["rts_before_us"].as<uint32_t>(cfg.rts_before_us);
        cfg.rts_after_us = root["rts_after_us"].as<uint32_t>(cfg.rts_after_us);
        cfg.poll_interval_ms = root["poll_interval_ms"].as<int>(cfg.poll_interval_ms);
        cfg.param_read_interval_s =
            root["param_read_interval_s"].as<int>(cfg.param_read_interval_s);
        cfg.max_consecutive_errors =
            root["max_consecutive_errors"].as<int>(cfg.max_consecutive_errors);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "battery_node: failed to load config " << filepath << ": "
                  << e.what() << std::endl;
        return false;
    }
}

void logProtectionChanges(uint16_t prev, uint16_t now) {
    uint16_t changed = static_cast<uint16_t>(prev ^ now);
    if (changed == 0) return;

    std::cerr << "battery_node: protection status change:" << std::endl;
    for (uint16_t bit = 1; bit <= bms::PROT_MOS_SOFT_LOCK; bit = static_cast<uint16_t>(bit << 1)) {
        if ((changed & bit) == 0) continue;
        std::cerr << "  " << bms::protection_bit_name(bit) << " "
                  << ((now & bit) ? "ACTIVE" : "cleared") << std::endl;
    }
}

void fillMessage(quad::battery_status_t& msg, const bms::BatteryStatus& s) {
    msg.pack_voltage_V = s.pack_voltage_V;
    msg.current_A = s.current_A;
    msg.remaining_capacity_Ah = s.remaining_capacity_Ah;
    msg.nominal_capacity_Ah = s.nominal_capacity_Ah;
    for (size_t i = 0; i < bms::kCellCount; ++i) {
        msg.cell_voltages[i] = s.cell_voltages_V[i];
    }
    msg.temp1_C = s.temps_C[0];
    msg.temp2_C = s.temps_C[1];
    msg.rsoc_percent = static_cast<int8_t>(s.rsoc_percent);
    msg.charge_fet_closed = s.mos_charge_closed;
    msg.discharge_fet_closed = s.mos_discharge_closed;
    msg.protection_bits = static_cast<int32_t>(s.protect_status);
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::string config_file = "config/battery.yaml";
    if (argc > 1) {
        config_file = argv[1];
    }

    BatteryNodeConfig cfg;
    if (!loadConfig(config_file, cfg)) {
        return -1;
    }

    lcm::LCM lcm;
    if (!lcm.good()) {
        std::cerr << "battery_node: LCM init failed" << std::endl;
        return -1;
    }

    auto transport = bms::make_posix_serial(cfg.port.c_str(), cfg.baud,
                                            cfg.rs485_direction.c_str(),
                                            cfg.rts_before_us, cfg.rts_after_us);
    bms::BmsDriver driver(std::move(transport), static_cast<uint8_t>(cfg.slave));

    // Verify the link with a few attempts before entering the loop.
    bms::BatteryStatus status;
    bool link_ok = false;
    for (int attempt = 1; attempt <= 3 && !g_stop; ++attempt) {
        if (driver.read_status(&status) == bms::Error::Ok) {
            link_ok = true;
            break;
        }
        std::cerr << "battery_node: link check attempt " << attempt
                  << " failed: " << bms::error_name(driver.last_error()) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    if (!link_ok) {
        std::cerr << "battery_node: BMS not responding on " << cfg.port << std::endl;
        return -1;
    }
    std::cout << "battery_node: BMS link ok (" << cfg.port << ", slave "
              << cfg.slave << ", rs485=" << cfg.rs485_direction << ")" << std::endl;

    quad::battery_status_t msg{};
    uint16_t prev_protect = 0;
    int consecutive_errors = 0;
    int64_t last_param_read = 0;

    std::cout << "battery_node started" << std::endl;

    while (!g_stop) {
        auto start = std::chrono::steady_clock::now();

        bms::Error err = driver.read_status(&status);
        if (err == bms::Error::Ok) {
            consecutive_errors = 0;

            logProtectionChanges(prev_protect, status.protect_status);
            prev_protect = status.protect_status;

            fillMessage(msg, status);
            msg.error_code = 0;
            msg.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
            lcm.publish("BATTERY_STATUS", &msg);
        } else {
            ++consecutive_errors;
            msg.error_code = static_cast<int8_t>(err);
            msg.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
            lcm.publish("BATTERY_STATUS", &msg);
            std::cerr << "battery_node: read_status failed: "
                      << bms::error_name(err) << " (consecutive: "
                      << consecutive_errors << ")" << std::endl;
            if (consecutive_errors >= cfg.max_consecutive_errors) {
                std::this_thread::sleep_for(std::chrono::seconds(5));
            }
        }

        // Slow background read of protection parameters (log only).
        auto now_s = std::chrono::duration_cast<std::chrono::seconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
        if (now_s - last_param_read >= cfg.param_read_interval_s) {
            last_param_read = now_s;
            bms::ProtectionParams params;
            if (driver.read_parameters(&params) == bms::Error::Ok) {
                std::cout << "battery_node: params cell_uvp=" << params.cell_uvp_mV
                          << "mV pack_uvp=" << params.pack_uvp_10mV * 10 << "mV"
                          << " chg_ocp=" << params.chg_ocp_10mA * 10 << "mA"
                          << " dis_ocp=" << params.dis_ocp_10mA * 10 << "mA"
                          << std::endl;
            }
        }

        std::this_thread::sleep_until(
            start + std::chrono::milliseconds(cfg.poll_interval_ms));
    }

    return 0;
}
