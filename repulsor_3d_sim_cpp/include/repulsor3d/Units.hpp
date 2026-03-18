#pragma once

#include <cmath>

namespace repulsor3d::units {

template <typename Tag>
class Scalar {
 public:
  constexpr Scalar() = default;
  explicit constexpr Scalar(const double value) : value_(value) {}

  constexpr double Value() const { return value_; }

  constexpr Scalar operator+(const Scalar rhs) const { return Scalar(value_ + rhs.value_); }
  constexpr Scalar operator-(const Scalar rhs) const { return Scalar(value_ - rhs.value_); }
  constexpr Scalar operator*(const double scale) const { return Scalar(value_ * scale); }
  constexpr Scalar operator/(const double scale) const { return Scalar(value_ / scale); }

 private:
  double value_ = 0.0;
};

struct MeterTag {};
struct RadianTag {};
struct DegreeTag {};
struct SecondsTag {};

using Meters = Scalar<MeterTag>;
using Radians = Scalar<RadianTag>;
using Degrees = Scalar<DegreeTag>;
using Seconds = Scalar<SecondsTag>;

inline constexpr Radians ToRadians(const Degrees d) {
  return Radians(d.Value() * 3.14159265358979323846 / 180.0);
}

inline constexpr Degrees ToDegrees(const Radians r) {
  return Degrees(r.Value() * 180.0 / 3.14159265358979323846);
}

inline constexpr double ClampNonNegative(const double value) {
  return value < 0.0 ? 0.0 : value;
}

}  // namespace repulsor3d::units

