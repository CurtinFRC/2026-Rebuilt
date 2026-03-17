#pragma once

#include <glm/mat4x4.hpp>

#include "repulsor3d/render/SceneFrame.hpp"

namespace repulsor3d {

void ApplyRenderEntityHierarchyAndCulling(RenderSceneFrame& frame, const glm::mat4& viewProjection);

}  // namespace repulsor3d
