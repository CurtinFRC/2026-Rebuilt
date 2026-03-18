#include "repulsor3d/render/ecs/Systems.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <glm/common.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/trigonometric.hpp>

#include "repulsor3d/render/ecs/SpatialIndex.hpp"

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

bool TryExtractFrustumAabb(const glm::mat4& viewProjection, SpatialAabb& out) {
  const glm::mat4 inverseVp = glm::inverse(viewProjection);
  if (!glm::all(glm::isfinite(inverseVp[0])) ||
      !glm::all(glm::isfinite(inverseVp[1])) ||
      !glm::all(glm::isfinite(inverseVp[2])) ||
      !glm::all(glm::isfinite(inverseVp[3]))) {
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

  if (!glm::all(glm::isfinite(minP)) || !glm::all(glm::isfinite(maxP))) {
    return false;
  }

  out.min = minP;
  out.max = maxP;
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
          radius = std::max(radius, glm::length(typed.scale) * 0.5F);
        } else if constexpr (std::is_same_v<T, OverlayLine>) {
          center = glm::vec3{0.0F, 0.0F, 0.0F};
          radius = 0.0F;
        }
      },
      entity.payload);

  return {center, radius};
}

}  // namespace

EntityCullingStats ApplyRenderEntityHierarchyAndCulling(RenderSceneFrame& frame, const glm::mat4& viewProjection) {
  EntityCullingStats stats;
  stats.totalEntities = static_cast<int>(frame.entities.size());

  if (frame.entities.empty()) {
    return stats;
  }

  std::unordered_map<std::string, size_t> entityIndexById;
  entityIndexById.reserve(frame.entities.size());
  for (size_t i = 0; i < frame.entities.size(); ++i) {
    if (!frame.entities[i].id.empty()) {
      entityIndexById[frame.entities[i].id] = i;
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

    const std::string& parentId = frame.entities[index].parentId;
    if (!parentId.empty()) {
      const auto parentIt = entityIndexById.find(parentId);
      if (parentIt != entityIndexById.end() && parentIt->second != index) {
        resolveTransform(parentIt->second);
        local = worldTransforms[parentIt->second] * local;
      }
    }

    worldTransforms[index] = local;
    visitState[index] = 2;
  };

  for (size_t i = 0; i < frame.entities.size(); ++i) {
    resolveTransform(i);
  }

  for (size_t i = 0; i < frame.entities.size(); ++i) {
    if (frame.entities[i].hasTransform || !frame.entities[i].parentId.empty()) {
      ApplyWorldTransformToPayload(worldTransforms[i], frame.entities[i].payload);
    }
  }

  const auto frustumPlanes = ExtractFrustumPlanes(viewProjection);
  SpatialAabb frustumAabb;
  const bool hasFrustumAabb = TryExtractFrustumAabb(viewProjection, frustumAabb);
  UniformGridSpatialIndex spatialIndex(2.5F);
  std::vector<SpatialSphere> cullableSpheres;
  cullableSpheres.reserve(frame.entities.size());
  for (std::size_t i = 0; i < frame.entities.size(); ++i) {
    const auto& entity = frame.entities[i];
    const bool isOverlay = entity.pass == RenderPass::Overlay || std::holds_alternative<OverlayLine>(entity.payload);
    if (isOverlay || !entity.culling.enabled) {
      continue;
    }
    const auto [center, radius] = ComputeEntityBounds(entity);
    SpatialSphere sphere{
        .center = center,
        .radius = radius,
        .entityIndex = i,
    };
    cullableSpheres.push_back(sphere);
    spatialIndex.Insert(sphere);
  }

  std::vector<std::size_t> broadPhaseCandidates;
  if (hasFrustumAabb) {
    spatialIndex.QueryAabb(frustumAabb, broadPhaseCandidates);
  }
  if (broadPhaseCandidates.empty()) {
    broadPhaseCandidates.reserve(cullableSpheres.size());
    for (const auto& sphere : cullableSpheres) {
      broadPhaseCandidates.push_back(sphere.entityIndex);
    }
  }
  stats.candidates = static_cast<int>(broadPhaseCandidates.size());

  std::unordered_map<std::size_t, std::pair<glm::vec3, float>> candidateBounds;
  candidateBounds.reserve(broadPhaseCandidates.size());
  for (const std::size_t index : broadPhaseCandidates) {
    if (index >= frame.entities.size()) {
      continue;
    }
    candidateBounds.emplace(index, ComputeEntityBounds(frame.entities[index]));
  }

  std::vector<RenderEntity> visible;
  visible.reserve(frame.entities.size());
  for (std::size_t i = 0; i < frame.entities.size(); ++i) {
    auto& entity = frame.entities[i];
    const bool isOverlay = entity.pass == RenderPass::Overlay || std::holds_alternative<OverlayLine>(entity.payload);
    if (isOverlay || !entity.culling.enabled) {
      visible.push_back(std::move(entity));
      continue;
    }

    const auto boundsIt = candidateBounds.find(i);
    if (boundsIt == candidateBounds.end()) {
      continue;
    }
    const auto [center, radius] = boundsIt->second;
    if (IsSphereVisible(frustumPlanes, center, radius)) {
      visible.push_back(std::move(entity));
    }
  }

  frame.entities = std::move(visible);
  stats.visible = static_cast<int>(frame.entities.size());
  stats.culled = std::max(0, stats.totalEntities - stats.visible);
  return stats;
}

}  // namespace repulsor3d
