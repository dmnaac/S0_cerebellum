#include <lcm/lcm-cpp.hpp>
#include <quad/imu_t.hpp>
#include <quad/joint_state_t.hpp>
#include <quad/user_cmd_t.hpp>

#include <chrono>
#include <iostream>
#include <thread>

bool received = false;

void handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
             const quad::user_cmd_t* msg) {
    std::cout << "Received on " << channel << ": vx=" << msg->vx
              << " mode=" << static_cast<int>(msg->mode) << std::endl;
    received = true;
}

int main(int argc, char** argv) {
    lcm::LCM lcm;
    if (!lcm.good()) {
        std::cerr << "LCM init failed" << std::endl;
        return -1;
    }

    lcm.subscribe("USER_CMD", &handler);

    quad::user_cmd_t cmd;
    cmd.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count();
    cmd.vx = 0.5;
    cmd.vy = 0.0;
    cmd.yaw_rate = 0.1;
    cmd.mode = 2;

    lcm.publish("USER_CMD", &cmd);

    auto start = std::chrono::steady_clock::now();
    while (!received && std::chrono::steady_clock::now() - start < std::chrono::seconds(1)) {
        lcm.handleTimeout(100);
    }

    return received ? 0 : -1;
}
