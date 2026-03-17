#pragma once

#include <functional>

namespace repulsor3d {

class FixedStepTicker {
 public:
  explicit FixedStepTicker(double fixedStepSeconds, double maxAccumulatorSeconds = 0.5);

  void Advance(double frameDeltaSeconds, const std::function<void(double)>& tickFn);

 private:
  double fixedStepSeconds_ = 1.0 / 60.0;
  double maxAccumulatorSeconds_ = 0.5;
  double accumulator_ = 0.0;
};

}  // namespace repulsor3d
