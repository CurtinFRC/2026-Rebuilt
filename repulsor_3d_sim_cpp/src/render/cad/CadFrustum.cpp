#include "repulsor3d/render/cad/CadFrustum.hpp"

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

}  // namespace repulsor3d::cad

