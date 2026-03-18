#include <cmath>
#include <iostream>
#include <optional>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/domain/SnapshotDomainAdapter.hpp"
#include "repulsor3d/sim/PoseSmoothing.hpp"

namespace {

bool NearlyEqual(const double a, const double b, const double eps = 1e-4) {
  return std::abs(a - b) <= eps;
}

int RunPoseSmoothingTests() {
  repulsor3d::ViewerConfig cfg;
  cfg.robotSmoothTimeS = 0.2F;
  cfg.robotMaxSpeedMps = 20.0F;

  repulsor3d::PoseSmoothingState state;

  const auto noneOut = repulsor3d::SmoothPose(std::nullopt, state, cfg, 0.02);
  if (noneOut.has_value()) {
    std::cerr << "Expected null pose when raw pose is null\n";
    return 1;
  }

  repulsor3d::Pose2D rawPose{
      .x = 1.0,
      .y = 2.0,
      .thetaRad = 0.3,
  };
  const auto first = repulsor3d::SmoothPose(rawPose, state, cfg, 0.02);
  if (!first.has_value() || !NearlyEqual(first->x, 1.0) || !NearlyEqual(first->y, 2.0)) {
    std::cerr << "First smoothing step should initialize directly to raw pose\n";
    return 2;
  }

  rawPose.x = 3.0;
  rawPose.y = 4.0;
  const auto second = repulsor3d::SmoothPose(rawPose, state, cfg, 0.02);
  if (!second.has_value()) {
    std::cerr << "Expected smoothed pose output\n";
    return 3;
  }
  if (!(second->x > 1.0 && second->x < 3.0 && second->y > 2.0 && second->y < 4.0)) {
    std::cerr << "Smoothed pose should move toward target, not jump immediately\n";
    return 4;
  }

  return 0;
}

int RunDomainAdapterTests() {
  repulsor3d::ViewerConfig cfg;
  cfg.incomingCoordFrame = "custom";
  cfg.incomingCoordOriginXM = 0.0F;
  cfg.incomingCoordOriginYM = 0.0F;
  cfg.incomingCoordRotationDeg = 0.0F;
  cfg.incomingCoordScaleMPerUnit = 1.0F;
  cfg.incomingCoordZScaleMPerUnit = 1.0F;
  repulsor3d::SnapshotDomainAdapter adapter(cfg);

  repulsor3d::SnapshotBundle bundle;
  bundle.snapshot.pose = repulsor3d::Pose2D{.x = 5.0, .y = 6.0, .thetaRad = 0.5};

  const auto renderSnap = adapter.BuildRenderSnapshot(bundle, 0.02);
  if (!renderSnap.pose.has_value()) {
    std::cerr << "Domain adapter should preserve pose presence\n";
    return 5;
  }

  const auto follow = adapter.ComputeDesiredFollowTarget(renderSnap, {0.0F, 0.0F, 0.0F});
  if (!NearlyEqual(follow.x, 5.0) || !NearlyEqual(follow.y, 6.0)) {
    std::cerr << "Domain adapter follow target mismatch\n";
    return 6;
  }

  return 0;
}

int RunCoordinateFrameMapperTests() {
  repulsor3d::ViewerConfig cfg;
  cfg.incomingCoordFrame = "top_right_negative";
  cfg.incomingCoordZScaleMPerUnit = 1.0F;

  repulsor3d::SnapshotDomainAdapter adapter(cfg);
  repulsor3d::SnapshotBundle bundle;
  bundle.snapshot.pose = repulsor3d::Pose2D{.x = 0.0, .y = 0.0, .thetaRad = 0.0};

  const auto renderSnap = adapter.BuildRenderSnapshot(bundle, 0.02);
  if (!renderSnap.pose.has_value()) {
    std::cerr << "Coordinate mapper test expected pose output\n";
    return 7;
  }
  const double halfLength = static_cast<double>(cfg.fieldLengthM) * 0.5;
  const double halfWidth = static_cast<double>(cfg.fieldWidthM) * 0.5;
  if (!NearlyEqual(renderSnap.pose->x, -halfLength) || !NearlyEqual(renderSnap.pose->y, -halfWidth)) {
    std::cerr << "Coordinate mapper top-right mismatch\n";
    return 8;
  }

  repulsor3d::SnapshotDomainAdapter bottomLeftAdapter(cfg);
  repulsor3d::SnapshotBundle bundleBottomLeft;
  bundleBottomLeft.snapshot.pose = repulsor3d::Pose2D{
      .x = -static_cast<double>(cfg.fieldLengthM),
      .y = -static_cast<double>(cfg.fieldWidthM),
      .thetaRad = 0.0};
  const auto renderBottomLeft = bottomLeftAdapter.BuildRenderSnapshot(bundleBottomLeft, 0.02);
  if (!renderBottomLeft.pose.has_value()) {
    std::cerr << "Coordinate mapper bottom-left expected pose output\n";
    return 9;
  }
  if (!NearlyEqual(renderBottomLeft.pose->x, halfLength) || !NearlyEqual(renderBottomLeft.pose->y, halfWidth)) {
    std::cerr << "Coordinate mapper bottom-left mismatch\n";
    return 10;
  }

  return 0;
}

}  // namespace

int main() {
  if (const int rc = RunPoseSmoothingTests(); rc != 0) {
    return rc;
  }
  if (const int rc = RunDomainAdapterTests(); rc != 0) {
    return rc;
  }
  if (const int rc = RunCoordinateFrameMapperTests(); rc != 0) {
    return rc;
  }

  std::cout << "All seam tests passed\n";
  return 0;
}
