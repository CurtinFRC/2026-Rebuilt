#pragma once

#if defined(REPULSOR_HAS_NTCORE)

#include <optional>
#include <string>

#include "repulsor3d/Model.hpp"
#include "repulsor3d/nt/SubscriberCollection.hpp"

namespace repulsor3d::nt {

std::optional<FieldVisionObject> TryMapFieldVisionObject(const std::string& id, const SubscriberCollection& subs);
std::optional<RepulsorVisionObstacle> TryMapRepulsorObstacle(const std::string& id, const SubscriberCollection& subs);
std::optional<CameraInfo> TryMapCameraInfo(const std::string& id, const SubscriberCollection& subs);

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)

