#pragma once

#include <glm/vec3.hpp>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/Model.hpp"
#include "repulsor3d/domain/CoordinateSystemService.hpp"
#include "repulsor3d/sim/PoseSmoothing.hpp"

namespace repulsor3d {

class SnapshotDomainAdapter {
 public:
  explicit SnapshotDomainAdapter(const ViewerConfig& cfg);

  WorldSnapshot BuildRenderSnapshot(const SnapshotBundle& latest, double dt);
  glm::vec3 ComputeDesiredFollowTarget(const WorldSnapshot& snapshot, const glm::vec3& fallback) const;
  void ApplyConfig(const ViewerConfig& cfg);
  void Reset();

 private:
  ViewerConfig cfg_;
  CoordinateSystemService coordinateSystem_;
  PoseSmoothingState poseState_;
};

}  // namespace repulsor3d
