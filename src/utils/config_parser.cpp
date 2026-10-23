#include "utils/config_parser.hpp"

#include <iostream>

namespace cerebellum {
namespace utils {

ConfigParser::ConfigParser(const std::string& filepath) : filepath_(filepath) {}

bool ConfigParser::load() {
    try {
        root_ = YAML::LoadFile(filepath_);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load config " << filepath_ << ": " << e.what() << std::endl;
        return false;
    }
}

YAML::Node ConfigParser::getRoot() const {
    return root_;
}

bool ConfigParser::has(const std::string& key) const {
    return root_[key];
}

} // namespace utils
} // namespace cerebellum
