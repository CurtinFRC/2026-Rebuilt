#include "repulsor3d/time/FixedStepTicker.hpp"

#include <algorithm>

namespace repulsor3d {

FixedStepTicker::FixedStepTicker(const double fixedStepSeconds, const double maxAccumulatorSeconds)
    : fixedStepSeconds_(fixedStepSeconds), maxAccumulatorSeconds_(maxAccumulatorSeconds) {}

void FixedStepTicker::Advance(const double frameDeltaSeconds, const std::function<void(double)>& tickFn) {
  accumulator_ = std::min(maxAccumulatorSeconds_, accumulator_ + frameDeltaSeconds);
  while (accumulator_ >= fixedStepSeconds_) {
    tickFn(fixedStepSeconds_);
    accumulator_ -= fixedStepSeconds_;
  }
}

void FixedStepTicker::SetFixedStepSeconds(const double fixedStepSeconds) {
  fixedStepSeconds_ = std::max(1e-4, fixedStepSeconds);
  accumulator_ = std::min(accumulator_, maxAccumulatorSeconds_);
}

}  // namespace repulsor3d
