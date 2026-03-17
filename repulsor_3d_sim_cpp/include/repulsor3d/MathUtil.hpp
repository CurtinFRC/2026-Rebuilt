#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace repulsor3d {

inline double Clamp(const double v, const double lo, const double hi) {
  return std::max(lo, std::min(v, hi));
}

inline double WrapPi(const double rad) {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTau = 2.0 * kPi;
  const double wrapped = std::fmod(rad + kPi, kTau);
  return (wrapped < 0.0 ? wrapped + kTau : wrapped) - kPi;
}

inline std::pair<double, double> SmoothDamp(
    const double current,
    const double target,
    const double currentVelocity,
    const double smoothTime,
    const double dt,
    const double maxSpeed = std::numeric_limits<double>::infinity()) {
  const double st = std::max(1e-4, smoothTime);
  const double clampedDt = std::max(0.0, dt);

  const double omega = 2.0 / st;
  const double x = omega * clampedDt;
  const double exp = 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x);

  double change = current - target;
  const double originalTarget = target;

  if (std::isfinite(maxSpeed)) {
    const double maxChange = maxSpeed * st;
    change = Clamp(change, -maxChange, maxChange);
  }

  const double adjustedTarget = current - change;
  const double temp = (currentVelocity + omega * change) * clampedDt;

  double newVelocity = (currentVelocity - omega * temp) * exp;
  double newValue = adjustedTarget + (change + temp) * exp;

  if ((originalTarget - current > 0.0) == (newValue > originalTarget)) {
    newValue = originalTarget;
    newVelocity = 0.0;
  }

  return {newValue, newVelocity};
}

inline std::pair<double, double> SmoothDampAngle(
    const double current,
    const double target,
    const double currentVelocity,
    const double smoothTime,
    const double dt,
    const double maxSpeed = std::numeric_limits<double>::infinity()) {
  const double delta = WrapPi(target - current);
  const double unwrappedTarget = current + delta;
  auto [value, velocity] = SmoothDamp(current, unwrappedTarget, currentVelocity, smoothTime, dt, maxSpeed);
  return {WrapPi(value), velocity};
}

}  // namespace repulsor3d
