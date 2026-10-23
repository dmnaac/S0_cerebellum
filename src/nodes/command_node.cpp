#include <lcm/lcm-cpp.hpp>
#include <quad/user_cmd_t.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

// TODO: add joystick / keyboard / remote input parsing

struct CommandContext {
    lcm::LCM lcm;
    quad::user_cmd_t cmd;
};

// Demo state sequence for testing
std::string getDemoState(double t) {
    int phase = static_cast<int>(t / 5.0) % 6;
    switch (phase) {
        case 0: return "stand";
        case 1: return "crouch";
        case 2: return "stand";
        case 3: return "trot";
        case 4: return "stand";
        case 5: return "lie";
        default: return "stand";
    }
}

void updateCommand(quad::user_cmd_t& cmd) {
    // Placeholder: read from joystick or keyboard
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    double t = std::chrono::duration<double>(now).count();

    cmd.vx = 0.3 * std::sin(0.5 * t);
    cmd.vy = 0.0;
    cmd.yaw_rate = 0.0;
    cmd.state_name = getDemoState(t);
    cmd.mode = 2; // walk, deprecated
}

int main(int argc, char** argv) {
    CommandContext ctx;
    if (!ctx.lcm.good()) {
        std::cerr << "command_node: LCM init failed" << std::endl;
        return -1;
    }

    ctx.cmd.vx = 0.0;
    ctx.cmd.vy = 0.0;
    ctx.cmd.yaw_rate = 0.0;
    ctx.cmd.state_name = "stand";
    ctx.cmd.mode = 1;

    std::cout << "command_node started" << std::endl;

    while (true) {
        auto start = std::chrono::steady_clock::now();

        updateCommand(ctx.cmd);
        ctx.cmd.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
        ctx.lcm.publish("USER_CMD", &ctx.cmd);

        std::this_thread::sleep_until(start + std::chrono::milliseconds(20));
    }

    return 0;
}
