#pragma once

#include <array>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace repulsor3d::cad {

struct Plane {
  glm::vec3 normal{0.0F, 0.0F, 1.0F};
  float d = 0.0F;
};

struct Aabb {
  glm::vec3 min{0.0F, 0.0F, 0.0F};
  glm::vec3 max{0.0F, 0.0F, 0.0F};
};

std::array<Plane, 6> ExtractFrustumPlanes(const glm::mat4& viewProjection);
bool IsSphereVisible(const std::array<Plane, 6>& planes, const glm::vec3& center, float radius, float margin = 0.0F);
bool TryExtractFrustumAabb(const glm::mat4& viewProjection, Aabb& out);

}  // namespace repulsor3d::cad
