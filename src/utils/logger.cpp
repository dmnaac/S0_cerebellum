#include "utils/logger.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>

namespace cerebellum {
namespace utils {

Logger::Logger(const std::string& filename) : filename_(filename) {}

Logger::~Logger() {
    close();
}

bool Logger::open() {
    std::lock_guard<std::mutex> lock(mutex_);
    file_.open(filename_, std::ios::out | std::ios::app);
    return file_.is_open();
}

void Logger::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open()) {
        file_.close();
    }
}

void Logger::log(const std::string& level, const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!file_.is_open()) return;

    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    file_ << std::put_time(std::localtime(&t), "%Y-%m-%d %H:%M:%S");
    file_ << " [" << level << "] " << message << std::endl;
}

void Logger::info(const std::string& message) {
    log("INFO", message);
}

void Logger::warn(const std::string& message) {
    log("WARN", message);
}

void Logger::error(const std::string& message) {
    log("ERROR", message);
}

} // namespace utils
} // namespace cerebellum
