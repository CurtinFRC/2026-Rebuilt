#if defined(REPULSOR_HAS_NTCORE)

#include "repulsor3d/nt/PoseBinding.hpp"

#include <cmath>

#include "repulsor3d/nt/TopicPath.hpp"

namespace repulsor3d::nt {
namespace {

bool IsFinite(const double value) {
  return std::isfinite(value) != 0;
}

}  // namespace

PoseBinding::PoseBinding(::nt::NetworkTableInstance* instance) : subs_(instance) {}

void PoseBinding::SetInstance(::nt::NetworkTableInstance* instance) {
  subs_.SetInstance(instance);
}

void PoseBinding::BindPose2D(
    const std::string& rootTopic,
    const std::string& xSuffix,
    const std::string& ySuffix,
    const std::string& thetaSuffix) {
  subs_.AddDouble("x", JoinTopic(rootTopic, xSuffix));
  subs_.AddDouble("y", JoinTopic(rootTopic, ySuffix));
  subs_.AddDouble("theta", JoinTopic(rootTopic, thetaSuffix));
}

std::optional<Pose2D> PoseBinding::ReadPose2D() const {
  if (!subs_.ContainsKey("x") || !subs_.ContainsKey("y") || !subs_.ContainsKey("theta")) {
    return std::nullopt;
  }

  const double x = subs_.GetDouble("x");
  const double y = subs_.GetDouble("y");
  const double theta = subs_.GetDouble("theta");
  if (!IsFinite(x) || !IsFinite(y) || !IsFinite(theta)) {
    return std::nullopt;
  }
  return Pose2D{x, y, theta};
}

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)

