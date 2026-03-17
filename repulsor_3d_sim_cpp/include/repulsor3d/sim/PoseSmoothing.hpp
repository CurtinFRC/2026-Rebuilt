#pragma once

#include <optional>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/Model.hpp"

namespace repulsor3d {

struct PoseSmoothingState {
  std::optional<double> x;
  std::optional<double> y;
  std::optional<double> heading;
  double vx = 0.0;
  double vy = 0.0;
  double vh = 0.0;
};

void ResetPoseSmoothing(PoseSmoothingState& state);
std::optional<Pose2D> SmoothPose(
    const std::optional<Pose2D>& rawPose,
    PoseSmoothingState& state,
    const ViewerConfig& cfg,
    double dt);

}  // namespace repulsor3d
