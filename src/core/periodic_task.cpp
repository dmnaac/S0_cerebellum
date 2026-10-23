#include "core/periodic_task.hpp"

#include <chrono>
#include <iostream>
#include <thread>

namespace cerebellum {
namespace core {

PeriodicTask::PeriodicTask(const std::string& name, double frequency_hz, Callback cb)
    : name_(name), period_us_(1e6 / frequency_hz), callback_(std::move(cb)) {}

PeriodicTask::~PeriodicTask() {
    stop();
}

bool PeriodicTask::start() {
    if (running_.exchange(true)) {
        return false;
    }
    thread_ = std::thread(&PeriodicTask::loop, this);
    return true;
}

void PeriodicTask::stop() {
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
}

bool PeriodicTask::isRunning() const {
    return running_;
}

double PeriodicTask::getAverageDt() const {
    return avg_dt_us_;
}

void PeriodicTask::loop() {
    using namespace std::chrono;
    auto next_time = steady_clock::now();
    const auto period = microseconds(static_cast<int64_t>(period_us_));

    while (running_) {
        auto start = steady_clock::now();
        if (callback_) {
            callback_();
        }
        auto end = steady_clock::now();
        avg_dt_us_ = duration<double, std::micro>(end - start).count();

        next_time += period;
        std::this_thread::sleep_until(next_time);
    }
}

} // namespace core
} // namespace cerebellum
