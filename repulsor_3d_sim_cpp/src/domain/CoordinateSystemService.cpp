#include "repulsor3d/domain/CoordinateSystemService.hpp"

namespace repulsor3d {

void CoordinateSystemService::SetTransform(
    const CoordinateFrameId from,
    const CoordinateFrameId to,
    CoordinateFrameMapper::Config config) {
  mappers_.insert_or_assign(TransformKey{from, to}, CoordinateFrameMapper(config));
}

bool CoordinateSystemService::HasTransform(const CoordinateFrameId from, const CoordinateFrameId to) const {
  return FindMapper(from, to) != nullptr;
}

glm::vec2 CoordinateSystemService::TransformPointXY(
    const CoordinateFrameId from,
    const CoordinateFrameId to,
    const double x,
    const double y) const {
  if (const auto* mapper = FindMapper(from, to); mapper != nullptr) {
    return mapper->TransformPointXY(x, y);
  }
  return glm::vec2{static_cast<float>(x), static_cast<float>(y)};
}

double CoordinateSystemService::TransformZ(
    const CoordinateFrameId from,
    const CoordinateFrameId to,
    const double z) const {
  if (const auto* mapper = FindMapper(from, to); mapper != nullptr) {
    return mapper->TransformZ(z);
  }
  return z;
}

Pose2D CoordinateSystemService::TransformPose(
    const CoordinateFrameId from,
    const CoordinateFrameId to,
    const Pose2D& pose) const {
  if (const auto* mapper = FindMapper(from, to); mapper != nullptr) {
    return mapper->TransformPose(pose);
  }
  return pose;
}

void CoordinateSystemService::TransformSnapshot(
    const CoordinateFrameId from,
    const CoordinateFrameId to,
    WorldSnapshot& snapshot) const {
  if (const auto* mapper = FindMapper(from, to); mapper != nullptr) {
    mapper->TransformSnapshot(snapshot);
  }
}

const CoordinateFrameMapper* CoordinateSystemService::FindMapper(
    const CoordinateFrameId from,
    const CoordinateFrameId to) const {
  const auto it = mappers_.find(TransformKey{from, to});
  if (it == mappers_.end()) {
    return nullptr;
  }
  return &it->second;
}

}  // namespace repulsor3d
