#pragma once

#include <glm/mat4x4.hpp>

#include "repulsor3d/render/SceneFrame.hpp"

namespace repulsor3d {

struct EntityCullingStats {
  int totalEntities = 0;
  int candidates = 0;
  int visible = 0;
  int culled = 0;
  double hierarchyMs = 0.0;
  double transformApplyMs = 0.0;
  double boundsBuildMs = 0.0;
  double visibilityTestMs = 0.0;
};

EntityCullingStats ApplyRenderEntityHierarchyAndCulling(RenderSceneFrame& frame, const glm::mat4& viewProjection);

}  // namespace repulsor3d
