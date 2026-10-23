#include "core/lowpass_filter.hpp"

#include <cmath>

namespace cerebellum {
namespace core {

LowpassFilter::LowpassFilter(double cutoff_hz, double dt) : value_(0.0) {
    double rc = 1.0 / (2.0 * M_PI * cutoff_hz);
    alpha_ = dt / (rc + dt);
}

double LowpassFilter::update(double input) {
    value_ += alpha_ * (input - value_);
    return value_;
}

double LowpassFilter::getValue() const {
    return value_;
}

void LowpassFilter::reset(double value) {
    value_ = value;
}

} // namespace core
} // namespace cerebellum
