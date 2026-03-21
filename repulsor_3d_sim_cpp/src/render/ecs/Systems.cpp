#include "repulsor3d/render/ecs/Systems.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <functional>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <glm/common.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/trigonometric.hpp>

#include "repulsor3d/render/MeshCulling.hpp"

namespace repulsor3d {
namespace {

struct Plane {
  glm::vec3 normal{0.0F, 0.0F, 1.0F};
  float d = 0.0F;
};

glm::mat4 ComposeTransform(const Transform3D& transform) {
  glm::mat4 out(1.0F);
  out = glm::translate(out, transform.position);
  out = glm::rotate(out, glm::radians(transform.rotationDeg.x), glm::vec3{1.0F, 0.0F, 0.0F});
  out = glm::rotate(out, glm::radians(transform.rotationDeg.y), glm::vec3{0.0F, 1.0F, 0.0F});
  out = glm::rotate(out, glm::radians(transform.rotationDeg.z), glm::vec3{0.0F, 0.0F, 1.0F});
  out = glm::scale(out, transform.scale);
  return out;
}

glm::vec3 TransformPoint(const glm::mat4& matrix, const glm::vec3& point) {
  const glm::vec4 transformed = matrix * glm::vec4{point, 1.0F};
  if (std::abs(transformed.w) > 1e-6F) {
    return glm::vec3{transformed} / transformed.w;
  }
  return glm::vec3{transformed};
}

glm::vec3 ExtractScale(const glm::mat4& matrix) {
  const glm::vec3 xAxis{matrix[0][0], matrix[0][1], matrix[0][2]};
  const glm::vec3 yAxis{matrix[1][0], matrix[1][1], matrix[1][2]};
  const glm::vec3 zAxis{matrix[2][0], matrix[2][1], matrix[2][2]};
  return glm::vec3{glm::length(xAxis), glm::length(yAxis), glm::length(zAxis)};
}

float ExtractYawDeg(const glm::mat4& matrix) {
  const float yawRad = std::atan2(matrix[0][1], matrix[0][0]);
  return glm::degrees(yawRad);
}

Plane NormalizePlane(const glm::vec4& plane) {
  const glm::vec3 n{plane.x, plane.y, plane.z};
  const float len = glm::length(n);
  if (len <= 1e-6F) {
    return {};
  }
  return {n / len, plane.w / len};
}

std::array<Plane, 6> ExtractFrustumPlanes(const glm::mat4& viewProjection) {
  const glm::vec4 row0{viewProjection[0][0], viewProjection[1][0], viewProjection[2][0], viewProjection[3][0]};
  const glm::vec4 row1{viewProjection[0][1], viewProjection[1][1], viewProjection[2][1], viewProjection[3][1]};
  const glm::vec4 row2{viewProjection[0][2], viewProjection[1][2], viewProjection[2][2], viewProjection[3][2]};
  const glm::vec4 row3{viewProjection[0][3], viewProjection[1][3], viewProjection[2][3], viewProjection[3][3]};

  return {
      NormalizePlane(row3 + row0),  // left
      NormalizePlane(row3 - row0),  // right
      NormalizePlane(row3 + row1),  // bottom
      NormalizePlane(row3 - row1),  // top
      NormalizePlane(row3 + row2),  // near
      NormalizePlane(row3 - row2),  // far
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

void ApplyWorldTransformToPayload(const glm::mat4& world, RenderEntityPayload& payload) {
  const glm::vec3 worldScale = ExtractScale(world);
  const float maxScale = std::max(worldScale.x, std::max(worldScale.y, worldScale.z));
  const float yawDeg = ExtractYawDeg(world);

  std::visit(
      [&](auto& typed) {
        using T = std::decay_t<decltype(typed)>;
        if constexpr (std::is_same_v<T, SpherePrimitive>) {
          typed.center = TransformPoint(world, typed.center);
          typed.radius *= std::max(0.0001F, maxScale);
        } else if constexpr (std::is_same_v<T, BoxPrimitive>) {
          typed.center = TransformPoint(world, typed.center);
          typed.yawDeg += yawDeg;
          typed.size *= glm::max(worldScale, glm::vec3{0.0001F, 0.0001F, 0.0001F});
        } else if constexpr (std::is_same_v<T, LinePrimitive>) {
          typed.a = TransformPoint(world, typed.a);
          typed.b = TransformPoint(world, typed.b);
        } else if constexpr (std::is_same_v<T, MeshInstancePrimitive>) {
          typed.position = TransformPoint(world, typed.position);
          typed.rotationDeg.z += yawDeg;
          typed.scale *= glm::max(worldScale, glm::vec3{0.0001F, 0.0001F, 0.0001F});
        } else if constexpr (std::is_same_v<T, OverlayLine>) {
          // Overlay is screen space and intentionally ignores world transforms.
        }
      },
      payload);
}

std::pair<glm::vec3, float> ComputeEntityBounds(const RenderEntity& entity) {
  glm::vec3 center{0.0F, 0.0F, 0.0F};
  float radius = std::max(0.001F, entity.culling.boundsRadius);

  std::visit(
      [&](const auto& typed) {
        using T = std::decay_t<decltype(typed)>;
        if constexpr (std::is_same_v<T, SpherePrimitive>) {
          center = typed.center;
          radius = std::max(radius, typed.radius);
        } else if constexpr (std::is_same_v<T, BoxPrimitive>) {
          center = typed.center;
          radius = std::max(radius, glm::length(typed.size) * 0.5F);
        } else if constexpr (std::is_same_v<T, LinePrimitive>) {
          center = (typed.a + typed.b) * 0.5F;
          radius = std::max(radius, glm::length(typed.b - typed.a) * 0.5F);
        } else if constexpr (std::is_same_v<T, MeshInstancePrimitive>) {
          center = typed.position;
          radius = std::max(radius, meshculling::ComputeMeshScaleBoundsRadius(typed.scale, 0.20F, 0.35F));
        } else if constexpr (std::is_same_v<T, OverlayLine>) {
          center = glm::vec3{0.0F, 0.0F, 0.0F};
          radius = 0.0F;
        }
      },
      entity.payload);

  return {center, radius};
}

bool ShouldApplyFrustumCulling(const RenderEntity& entity) {
  return entity.culling.enabled;
}

}  // namespace

EntityCullingStats ApplyRenderEntityHierarchyAndCulling(RenderSceneFrame& frame, const glm::mat4& viewProjection) {
  EntityCullingStats stats;
  stats.totalEntities = static_cast<int>(frame.entities.size());
  const auto statsStart = std::chrono::steady_clock::now();

  if (frame.entities.empty()) {
    return stats;
  }

  std::vector<int> parentIndices(frame.entities.size(), -1);
  for (size_t i = 0; i < frame.entities.size(); ++i) {
    if (frame.entities[i].parentId.empty()) {
      continue;
    }
    for (size_t j = 0; j < frame.entities.size(); ++j) {
      if (i == j) {
        continue;
      }
      if (frame.entities[j].id == frame.entities[i].parentId) {
        parentIndices[i] = static_cast<int>(j);
        break;
      }
    }
  }

  std::vector<glm::mat4> worldTransforms(frame.entities.size(), glm::mat4(1.0F));
  std::vector<int> visitState(frame.entities.size(), 0);

  std::function<void(size_t)> resolveTransform = [&](const size_t index) {
    if (index >= frame.entities.size()) {
      return;
    }
    if (visitState[index] == 2) {
      return;
    }
    if (visitState[index] == 1) {
      worldTransforms[index] = glm::mat4(1.0F);
      visitState[index] = 2;
      return;
    }

    visitState[index] = 1;
    glm::mat4 local = frame.entities[index].hasTransform
                          ? ComposeTransform(frame.entities[index].transform)
                          : glm::mat4(1.0F);

    const int parentIndex = parentIndices[index];
    if (parentIndex >= 0 && static_cast<size_t>(parentIndex) != index) {
      const size_t parent = static_cast<size_t>(parentIndex);
      resolveTransform(parent);
      local = worldTransforms[parent] * local;
    }

    worldTransforms[index] = local;
    visitState[index] = 2;
  };

  for (size_t i = 0; i < frame.entities.size(); ++i) {
    resolveTransform(i);
  }
  const auto afterHierarchy = std::chrono::steady_clock::now();

  for (size_t i = 0; i < frame.entities.size(); ++i) {
    if (frame.entities[i].hasTransform || !frame.entities[i].parentId.empty()) {
      ApplyWorldTransformToPayload(worldTransforms[i], frame.entities[i].payload);
    }
  }
  const auto afterTransformApply = std::chrono::steady_clock::now();

  const auto frustumPlanes = ExtractFrustumPlanes(viewProjection);
  std::vector<std::uint8_t> candidateMask(frame.entities.size(), 0U);
  std::vector<glm::vec3> boundsCenters(frame.entities.size(), glm::vec3{0.0F, 0.0F, 0.0F});
  std::vector<float> boundsRadii(frame.entities.size(), 0.0F);
  for (std::size_t i = 0; i < frame.entities.size(); ++i) {
    const auto& entity = frame.entities[i];
    const bool isOverlay = entity.pass == RenderPass::Overlay || std::holds_alternative<OverlayLine>(entity.payload);
    if (isOverlay || !ShouldApplyFrustumCulling(entity)) {
      continue;
    }
    const auto [center, radius] = ComputeEntityBounds(entity);
    boundsCenters[i] = center;
    boundsRadii[i] = radius;
    candidateMask[i] = 1U;
    ++stats.candidates;
  }
  const auto afterBoundsBuild = std::chrono::steady_clock::now();

  std::vector<RenderEntity> visible;
  visible.reserve(frame.entities.size());
  for (std::size_t i = 0; i < frame.entities.size(); ++i) {
    auto& entity = frame.entities[i];
    const bool isOverlay = entity.pass == RenderPass::Overlay || std::holds_alternative<OverlayLine>(entity.payload);
    if (isOverlay || !ShouldApplyFrustumCulling(entity)) {
      visible.push_back(std::move(entity));
      continue;
    }

    if (candidateMask[i] == 0U) {
      continue;
    }
    const glm::vec3& center = boundsCenters[i];
    const float radius = boundsRadii[i];
    if (IsSphereVisible(frustumPlanes, center, radius)) {
      visible.push_back(std::move(entity));
    }
  }

  frame.entities = std::move(visible);
  stats.visible = static_cast<int>(frame.entities.size());
  stats.culled = std::max(0, stats.totalEntities - stats.visible);
  const auto statsEnd = std::chrono::steady_clock::now();
  stats.hierarchyMs =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(afterHierarchy - statsStart).count();
  stats.transformApplyMs =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(afterTransformApply - afterHierarchy).count();
  stats.boundsBuildMs =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(afterBoundsBuild - afterTransformApply).count();
  stats.visibilityTestMs =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(statsEnd - afterBoundsBuild).count();
  return stats;
}

}  // namespace repulsor3d
