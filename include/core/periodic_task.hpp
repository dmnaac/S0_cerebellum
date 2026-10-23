#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <thread>

namespace cerebellum {
namespace core {

class PeriodicTask {
public:
    using Callback = std::function<void()>;

    PeriodicTask(const std::string& name, double frequency_hz, Callback cb);
    ~PeriodicTask();

    bool start();
    void stop();
    bool isRunning() const;
    double getAverageDt() const;

private:
    void loop();

    std::string name_;
    double period_us_;
    Callback callback_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::atomic<double> avg_dt_us_{0.0};
};

} // namespace core
} // namespace cerebellum
