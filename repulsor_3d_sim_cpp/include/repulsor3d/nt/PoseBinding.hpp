#pragma once

#if defined(REPULSOR_HAS_NTCORE)

#include <optional>
#include <string>

#include <networktables/NetworkTableInstance.h>

#include "repulsor3d/Model.hpp"
#include "repulsor3d/nt/SubscriberCollection.hpp"

namespace repulsor3d::nt {

class PoseBinding {
 public:
  explicit PoseBinding(::nt::NetworkTableInstance* instance = nullptr);

  void SetInstance(::nt::NetworkTableInstance* instance);

  void BindPose2D(
      const std::string& rootTopic,
      const std::string& xSuffix = "translation/x",
      const std::string& ySuffix = "translation/y",
      const std::string& thetaSuffix = "rotation/value");

  std::optional<Pose2D> ReadPose2D() const;

 private:
  SubscriberCollection subs_;
};

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)

