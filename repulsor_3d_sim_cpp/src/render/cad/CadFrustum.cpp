#include "repulsor3d/render/cad/CadFrustum.hpp"

#include <array>
#include <cmath>
#include <limits>

#include <glm/common.hpp>
#include <glm/geometric.hpp>

namespace repulsor3d::cad {
namespace {

Plane NormalizePlane(const glm::vec4& plane) {
  const glm::vec3 n{plane.x, plane.y, plane.z};
  const float len = glm::length(n);
  if (len <= 1e-6F) {
    return {};
  }
  return {n / len, plane.w / len};
}

bool IsFiniteVec3(const glm::vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool IsFiniteVec4(const glm::vec4& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) && std::isfinite(v.w);
}

}  // namespace

std::array<Plane, 6> ExtractFrustumPlanes(const glm::mat4& viewProjection) {
  const glm::vec4 row0{viewProjection[0][0], viewProjection[1][0], viewProjection[2][0], viewProjection[3][0]};
  const glm::vec4 row1{viewProjection[0][1], viewProjection[1][1], viewProjection[2][1], viewProjection[3][1]};
  const glm::vec4 row2{viewProjection[0][2], viewProjection[1][2], viewProjection[2][2], viewProjection[3][2]};
  const glm::vec4 row3{viewProjection[0][3], viewProjection[1][3], viewProjection[2][3], viewProjection[3][3]};
  return {
      NormalizePlane(row3 + row0),
      NormalizePlane(row3 - row0),
      NormalizePlane(row3 + row1),
      NormalizePlane(row3 - row1),
      NormalizePlane(row3 + row2),
      NormalizePlane(row3 - row2),
  };
}

bool IsSphereVisible(const std::array<Plane, 6>& planes, const glm::vec3& center, const float radius) {
  for (const auto& plane : planes) {
    const float distance = glm::dot(plane.normal, center) + plane.d;
    if (distance < -radius) {
      return false;
    }
  }
  return true;
}

bool TryExtractFrustumAabb(const glm::mat4& viewProjection, Aabb& out) {
  const glm::mat4 inverseVp = glm::inverse(viewProjection);
  if (!IsFiniteVec4(inverseVp[0]) ||
      !IsFiniteVec4(inverseVp[1]) ||
      !IsFiniteVec4(inverseVp[2]) ||
      !IsFiniteVec4(inverseVp[3])) {
    return false;
  }

  constexpr std::array<glm::vec3, 8> clipCorners{
      glm::vec3{-1.0F, -1.0F, -1.0F},
      glm::vec3{1.0F, -1.0F, -1.0F},
      glm::vec3{-1.0F, 1.0F, -1.0F},
      glm::vec3{1.0F, 1.0F, -1.0F},
      glm::vec3{-1.0F, -1.0F, 1.0F},
      glm::vec3{1.0F, -1.0F, 1.0F},
      glm::vec3{-1.0F, 1.0F, 1.0F},
      glm::vec3{1.0F, 1.0F, 1.0F},
  };

  glm::vec3 minP{std::numeric_limits<float>::max()};
  glm::vec3 maxP{std::numeric_limits<float>::lowest()};
  for (const glm::vec3& clip : clipCorners) {
    const glm::vec4 worldH = inverseVp * glm::vec4{clip, 1.0F};
    if (std::abs(worldH.w) <= 1e-6F) {
      continue;
    }
    const glm::vec3 world = glm::vec3{worldH} / worldH.w;
    minP = glm::min(minP, world);
    maxP = glm::max(maxP, world);
  }

  if (!IsFiniteVec3(minP) || !IsFiniteVec3(maxP)) {
    return false;
  }
  out.min = minP;
  out.max = maxP;
  return true;
}

}  // namespace repulsor3d::cad
