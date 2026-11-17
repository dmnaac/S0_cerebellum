#include <lcm/lcm-cpp.hpp>
#include <quad/power_cmd_t.hpp>
#include <quad/power_currents_t.hpp>
#include <quad/power_status_t.hpp>

#include "drivers/power_board.hpp"
#include "drivers/power_board_types.hpp"

#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace {

volatile std::sig_atomic_t g_stop = 0;

void onSignal(int) {
    g_stop = 1;
}

struct PowerNodeConfig {
    std::string can_iface = "can4";
    int power_id = 0xAA;
    std::string mode = "poll";  // "poll" or "auto"
    double poll_hz = 20.0;
    bool v12_on = true;
    bool vmbus_on = true;
    bool v24_on = false;
    int query_timeout_ms = 30;
};

bool loadConfig(const std::string& filepath, PowerNodeConfig& cfg) {
    try {
        YAML::Node root = YAML::LoadFile(filepath);
        cfg.can_iface = root["can_iface"].as<std::string>(cfg.can_iface);
        cfg.power_id = root["power_id"].as<int>(cfg.power_id);
        cfg.mode = root["mode"].as<std::string>(cfg.mode);
        cfg.poll_hz = root["poll_hz"].as<double>(cfg.poll_hz);
        cfg.v12_on = root["v12_on"].as<bool>(cfg.v12_on);
        cfg.vmbus_on = root["vmbus_on"].as<bool>(cfg.vmbus_on);
        cfg.v24_on = root["v24_on"].as<bool>(cfg.v24_on);
        cfg.query_timeout_ms = root["query_timeout_ms"].as<int>(cfg.query_timeout_ms);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "power_board_node: failed to load config " << filepath << ": "
                  << e.what() << std::endl;
        return false;
    }
}

} // namespace

struct PowerNodeContext {
    lcm::LCM lcm;
    cerebellum::drivers::PowerBoard* board = nullptr;
    PowerNodeConfig cfg;

    quad::power_status_t status_msg{};
    quad::power_currents_t currents_msg{};

    bool desired_v12;
    bool desired_vmbus;
    bool desired_v24;
    std::string version;

    uint64_t last_status_utime = 0;
};

void powerCmdHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                     const quad::power_cmd_t* msg, PowerNodeContext* ctx) {
    auto* board = ctx->board;
    if (!board) return;

    if (msg->v12 >= 0) ctx->desired_v12 = (msg->v12 == 1);
    if (msg->vmbus >= 0) ctx->desired_vmbus = (msg->vmbus == 1);
    if (msg->v24 >= 0) ctx->desired_v24 = (msg->v24 == 1);

    if (msg->restart) {
        std::cout << "power_board_node: restarting power board" << std::endl;
        board->restart();
    }
    if (msg->clear_fault) {
        std::cout << "power_board_node: clearing power board faults" << std::endl;
        board->clearFault();
    }
    board->setRails(ctx->desired_v12, ctx->desired_vmbus, ctx->desired_v24);
}

bool publishStatus(PowerNodeContext& ctx, const cerebellum::drivers::PowerStatus& s) {
    ctx.status_msg.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                               std::chrono::steady_clock::now().time_since_epoch())
                               .count();
    ctx.status_msg.battery_voltage = s.battery_voltage;
    ctx.status_msg.motor_voltage = s.motor_voltage;
    ctx.status_msg.bus_current = s.bus_current;
    ctx.status_msg.temperature = static_cast<int8_t>(s.temperature);
    ctx.status_msg.fault_bits = static_cast<int32_t>(s.fault_bits);
    ctx.status_msg.rail12v_on = s.rail12v_on;
    ctx.status_msg.softstart_on = s.softstart_on;
    ctx.status_msg.vmbus_on = s.vmbus_on;
    ctx.status_msg.rail24v_on = s.rail24v_on;
    ctx.status_msg.version = ctx.version;
    return ctx.lcm.publish("POWER_STATUS", &ctx.status_msg) == 0;
}

void publishCurrents(PowerNodeContext& ctx, const cerebellum::drivers::PowerCurrents& c) {
    ctx.currents_msg.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now().time_since_epoch())
                                 .count();
    ctx.currents_msg.al = c.al;
    ctx.currents_msg.ar = c.ar;
    ctx.currents_msg.ll = c.ll;
    ctx.currents_msg.lr = c.lr;
    ctx.lcm.publish("POWER_CURRENTS", &ctx.currents_msg);
}

void runPollLoop(PowerNodeContext& ctx) {
    auto* board = ctx.board;
    const auto period =
        std::chrono::microseconds(static_cast<int64_t>(1e6 / ctx.cfg.poll_hz));

    while (!g_stop) {
        auto start = std::chrono::steady_clock::now();
        ctx.lcm.handleTimeout(0);

        cerebellum::drivers::PowerStatus status;
        if (board->queryStatus(status, ctx.cfg.query_timeout_ms)) {
            publishStatus(ctx, status);
            ctx.last_status_utime = ctx.status_msg.utime;
        } else {
            std::cerr << "power_board_node: status query timeout" << std::endl;
        }

        cerebellum::drivers::PowerCurrents currents;
        if (board->queryCurrents(currents, ctx.cfg.query_timeout_ms)) {
            publishCurrents(ctx, currents);
        }

        std::this_thread::sleep_until(start + period);
    }
}

void runAutoLoop(PowerNodeContext& ctx) {
    auto* board = ctx.board;
    if (!board->setAutoReport(true)) {
        std::cerr << "power_board_node: failed to enable auto report, falling back to poll"
                  << std::endl;
        runPollLoop(ctx);
        return;
    }
    std::cout << "power_board_node: auto report enabled" << std::endl;

    while (!g_stop) {
        ctx.lcm.handleTimeout(0);

        cerebellum::drivers::CanFrame frame;
        while (board->receiveRaw(frame, 5)) {
            cerebellum::drivers::PowerStatus s;
            cerebellum::drivers::PowerCurrents c;
            if (cerebellum::drivers::PowerBoard::parseStatusFrame(frame, s)) {
                publishStatus(ctx, s);
                ctx.last_status_utime = ctx.status_msg.utime;
            } else if (cerebellum::drivers::PowerBoard::parseCurrentsFrame(frame, c)) {
                publishCurrents(ctx, c);
            }
        }
    }
    board->setAutoReport(false);
}

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::string config_file = "config/power.yaml";
    if (argc > 1) {
        config_file = argv[1];
    }

    PowerNodeContext ctx;
    if (!ctx.lcm.good()) {
        std::cerr << "power_board_node: LCM init failed" << std::endl;
        return -1;
    }
    if (!loadConfig(config_file, ctx.cfg)) {
        return -1;
    }

    ctx.desired_v12 = ctx.cfg.v12_on;
    ctx.desired_vmbus = ctx.cfg.vmbus_on;
    ctx.desired_v24 = ctx.cfg.v24_on;

    cerebellum::drivers::PowerBoard board(ctx.cfg.can_iface,
                                          static_cast<uint8_t>(ctx.cfg.power_id));
    ctx.board = &board;

    if (!board.open()) {
        std::cerr << "power_board_node: failed to open " << ctx.cfg.can_iface
                  << std::endl;
        return -1;
    }

    if (board.queryVersion(ctx.version, 200)) {
        std::cout << "power_board_node: firmware version: " << ctx.version << std::endl;
    } else {
        std::cerr << "power_board_node: version query timeout, continuing" << std::endl;
    }

    if (!board.setRails(ctx.desired_v12, ctx.desired_vmbus, ctx.desired_v24)) {
        std::cerr << "power_board_node: failed to set initial rail state" << std::endl;
        return -1;
    }
    std::cout << "power_board_node: rails set (12V=" << ctx.desired_v12
              << " VMBUS=" << ctx.desired_vmbus << " 24V=" << ctx.desired_v24 << ")"
              << std::endl;

    ctx.lcm.subscribe("POWER_CMD", &powerCmdHandler, &ctx);

    std::cout << "power_board_node started (mode=" << ctx.cfg.mode << ")" << std::endl;

    if (ctx.cfg.mode == "auto") {
        runAutoLoop(ctx);
    } else {
        runPollLoop(ctx);
    }

    return 0;
}
