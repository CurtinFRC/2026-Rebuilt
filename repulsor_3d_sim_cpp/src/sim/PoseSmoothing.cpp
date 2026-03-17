#include "repulsor3d/sim/PoseSmoothing.hpp"

#include <tuple>

#include "repulsor3d/MathUtil.hpp"

namespace repulsor3d {

void ResetPoseSmoothing(PoseSmoothingState& state) {
  state.x.reset();
  state.y.reset();
  state.heading.reset();
  state.vx = 0.0;
  state.vy = 0.0;
  state.vh = 0.0;
}

std::optional<Pose2D> SmoothPose(
    const std::optional<Pose2D>& rawPose,
    PoseSmoothingState& state,
    const ViewerConfig& cfg,
    const double dt) {
  if (!rawPose.has_value()) {
    ResetPoseSmoothing(state);
    return std::nullopt;
  }

  const Pose2D& p = rawPose.value();
  if (!state.x.has_value() || !state.y.has_value() || !state.heading.has_value()) {
    state.x = p.x;
    state.y = p.y;
    state.heading = p.thetaRad;
    state.vx = 0.0;
    state.vy = 0.0;
    state.vh = 0.0;
  } else {
    std::tie(*state.x, state.vx) =
        SmoothDamp(*state.x, p.x, state.vx, static_cast<double>(cfg.robotSmoothTimeS), dt, static_cast<double>(cfg.robotMaxSpeedMps));
    std::tie(*state.y, state.vy) =
        SmoothDamp(*state.y, p.y, state.vy, static_cast<double>(cfg.robotSmoothTimeS), dt, static_cast<double>(cfg.robotMaxSpeedMps));
    std::tie(*state.heading, state.vh) =
        SmoothDampAngle(*state.heading, p.thetaRad, state.vh, static_cast<double>(cfg.robotSmoothTimeS), dt, 50.0);
  }

  Pose2D out = p;
  out.x = *state.x;
  out.y = *state.y;
  out.thetaRad = *state.heading;
  return out;
}

}  // namespace repulsor3d
