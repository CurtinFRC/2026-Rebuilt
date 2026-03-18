#pragma once

#include <glm/mat4x4.hpp>

#include "repulsor3d/render/SceneFrame.hpp"

namespace repulsor3d {

struct EntityCullingStats {
  int totalEntities = 0;
  int candidates = 0;
  int visible = 0;
  int culled = 0;
};

EntityCullingStats ApplyRenderEntityHierarchyAndCulling(RenderSceneFrame& frame, const glm::mat4& viewProjection);

}  // namespace repulsor3d
