#pragma once

#include <algorithm>
#include <cmath>

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include "repulsor3d/render/SceneFrame.hpp"

namespace repulsor3d::meshculling {

inline constexpr float kPlaceholderBoundsRadius = 0.5F;

inline bool IsPlaceholderBoundsRadius(const float radius) {
  return radius <= (kPlaceholderBoundsRadius + 1e-4F);
}

inline float ComputeMeshScaleBoundsRadius(
    const glm::vec3& scale,
    const float paddingMeters = 0.25F,
    const float minRadiusMeters = 0.35F) {
  const glm::vec3 absScale = glm::abs(scale);
  const float halfDiagonal = glm::length(absScale) * 0.5F;
  return std::max(minRadiusMeters, halfDiagonal + std::max(0.0F, paddingMeters));
}

inline float ComputeRectFootprintBoundsRadius(
    const float lengthMeters,
    const float widthMeters,
    const float uniformScale = 1.0F,
    const float paddingMeters = 0.5F,
    const float minRadiusMeters = 0.5F) {
  const float absScale = std::max(0.0F, std::abs(uniformScale));
  const float halfLength = std::max(0.0F, lengthMeters * 0.5F * absScale);
  const float halfWidth = std::max(0.0F, widthMeters * 0.5F * absScale);
  const float diagonal = std::sqrt(halfLength * halfLength + halfWidth * halfWidth);
  return std::max(minRadiusMeters, diagonal + std::max(0.0F, paddingMeters));
}

inline EntityCulling ResolveMeshEntityCulling(
    const MeshInstancePrimitive& mesh,
    EntityCulling culling,
    const float paddingMeters = 0.25F,
    const float minRadiusMeters = 0.35F) {
  if (!culling.enabled) {
    return culling;
  }
  if (IsPlaceholderBoundsRadius(culling.boundsRadius)) {
    culling.boundsRadius = ComputeMeshScaleBoundsRadius(mesh.scale, paddingMeters, minRadiusMeters);
  }
  return culling;
}

}  // namespace repulsor3d::meshculling

