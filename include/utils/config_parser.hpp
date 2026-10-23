#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

namespace cerebellum {
namespace utils {

class ConfigParser {
public:
    explicit ConfigParser(const std::string& filepath);

    bool load();
    YAML::Node getRoot() const;
    bool has(const std::string& key) const;

private:
    std::string filepath_;
    YAML::Node root_;
};

} // namespace utils
} // namespace cerebellum
