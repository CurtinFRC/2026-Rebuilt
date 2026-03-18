#include "repulsor3d/domain/SnapshotDomainAdapter.hpp"

#include <algorithm>
#include <cctype>

#include "repulsor3d/domain/CoordinateCalibrationProfile.hpp"

namespace repulsor3d {
namespace {

std::string NormalizeLower(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return text;
}

CoordinateFrameMapper::Config BuildCoordinateMapperConfig(const ViewerConfig& cfg) {
  CoordinateFrameMapper::Config resolved;
  const std::string mode = NormalizeLower(cfg.incomingCoordFrame);
  if (mode == "top_right_negative") {
    resolved = CoordinateFrameMapper::Config{
        .originXM = -static_cast<double>(cfg.fieldLengthM) * 0.5,
        .originYM = -static_cast<double>(cfg.fieldWidthM) * 0.5,
        .rotationDeg = 180.0,
        .scaleMPerUnit = 1.0,
        .zScaleMPerUnit = std::max(1e-6, static_cast<double>(cfg.incomingCoordZScaleMPerUnit))};
    return ResolveCoordinateMapperConfigFromProfile(cfg, resolved);
  }

  resolved = CoordinateFrameMapper::Config{
      .originXM = static_cast<double>(cfg.incomingCoordOriginXM),
      .originYM = static_cast<double>(cfg.incomingCoordOriginYM),
      .rotationDeg = static_cast<double>(cfg.incomingCoordRotationDeg),
      .scaleMPerUnit = std::max(1e-6, static_cast<double>(cfg.incomingCoordScaleMPerUnit)),
      .zScaleMPerUnit = std::max(1e-6, static_cast<double>(cfg.incomingCoordZScaleMPerUnit))};
  return ResolveCoordinateMapperConfigFromProfile(cfg, resolved);
}

}  // namespace

SnapshotDomainAdapter::SnapshotDomainAdapter(const ViewerConfig& cfg)
    : cfg_(cfg) {
  coordinateSystem_.SetTransform(
      CoordinateFrameId::Incoming,
      CoordinateFrameId::Field,
      BuildCoordinateMapperConfig(cfg));
}

WorldSnapshot SnapshotDomainAdapter::BuildRenderSnapshot(const SnapshotBundle& latest, const double dt) {
  WorldSnapshot out = latest.snapshot;
  coordinateSystem_.TransformSnapshot(CoordinateFrameId::Incoming, CoordinateFrameId::Field, out);
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

void SnapshotDomainAdapter::ApplyConfig(const ViewerConfig& cfg) {
  cfg_ = cfg;
  coordinateSystem_.SetTransform(
      CoordinateFrameId::Incoming,
      CoordinateFrameId::Field,
      BuildCoordinateMapperConfig(cfg_));
}

void SnapshotDomainAdapter::Reset() {
  ResetPoseSmoothing(poseState_);
}

}  // namespace repulsor3d
