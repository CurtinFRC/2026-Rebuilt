#include "repulsor3d/domain/SnapshotDomainAdapter.hpp"

namespace repulsor3d {

SnapshotDomainAdapter::SnapshotDomainAdapter(const ViewerConfig& cfg)
    : cfg_(cfg),
      coordinateMapper_(CoordinateFrameMapper::Config{
          .originXM = static_cast<double>(cfg.incomingCoordOriginXM),
          .originYM = static_cast<double>(cfg.incomingCoordOriginYM),
          .rotationDeg = static_cast<double>(cfg.incomingCoordRotationDeg),
          .scaleMPerUnit = static_cast<double>(cfg.incomingCoordScaleMPerUnit),
          .zScaleMPerUnit = static_cast<double>(cfg.incomingCoordZScaleMPerUnit)}) {}

WorldSnapshot SnapshotDomainAdapter::BuildRenderSnapshot(const SnapshotBundle& latest, const double dt) {
  WorldSnapshot out = latest.snapshot;
  coordinateMapper_.TransformSnapshot(out);
  out.pose = SmoothPose(out.pose, poseState_, cfg_, dt);
  return out;
}

glm::vec3 SnapshotDomainAdapter::ComputeDesiredFollowTarget(const WorldSnapshot& snapshot, const glm::vec3& fallback) const {
  if (!snapshot.pose.has_value()) {
    return fallback;
  }
  return {
      static_cast<float>(snapshot.pose->x),
      static_cast<float>(snapshot.pose->y),
      0.0F,
  };
}

void SnapshotDomainAdapter::Reset() {
  ResetPoseSmoothing(poseState_);
}

}  // namespace repulsor3d
