#include <chrono>
#include <iostream>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/sim/PoseSmoothing.hpp"

namespace {

int RunPoseSmoothingPerf() {
  repulsor3d::ViewerConfig cfg;
  cfg.robotSmoothTimeS = 0.12F;
  cfg.robotMaxSpeedMps = 25.0F;

  repulsor3d::PoseSmoothingState state;
  repulsor3d::Pose2D pose{0.0, 0.0, 0.0};

  constexpr int kIterations = 50000;
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < kIterations; ++i) {
    pose.x += 0.001;
    pose.y += 0.001;
    pose.thetaRad += 0.0005;
    const auto out = repulsor3d::SmoothPose(pose, state, cfg, 0.005);
    if (!out.has_value()) {
      return 1;
    }
  }
  const auto end = std::chrono::steady_clock::now();
  const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  std::cout << "Pose smoothing " << kIterations << " iterations: " << elapsedMs << " ms\n";
  return 0;
}

}  // namespace

int main() {
  if (const int rc = RunPoseSmoothingPerf(); rc != 0) {
    return rc;
  }
  std::cout << "Performance smoke tests passed\n";
  return 0;
}
