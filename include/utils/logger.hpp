#pragma once

#include <fstream>
#include <mutex>
#include <string>

namespace cerebellum {
namespace utils {

class Logger {
public:
    explicit Logger(const std::string& filename);
    ~Logger();

    bool open();
    void close();

    void log(const std::string& level, const std::string& message);
    void info(const std::string& message);
    void warn(const std::string& message);
    void error(const std::string& message);

private:
    std::string filename_;
    std::ofstream file_;
    std::mutex mutex_;
};

} // namespace utils
} // namespace cerebellum
