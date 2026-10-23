#pragma once

namespace cerebellum {
namespace core {

class LowpassFilter {
public:
    explicit LowpassFilter(double cutoff_hz, double dt);

    double update(double input);
    double getValue() const;
    void reset(double value = 0.0);

private:
    double alpha_;
    double value_;
};

} // namespace core
} // namespace cerebellum
