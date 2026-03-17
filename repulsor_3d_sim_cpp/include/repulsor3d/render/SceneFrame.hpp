#pragma once

#include <string>
#include <variant>
#include <vector>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace repulsor3d {

enum class RenderPass {
  Background = 0,
  Opaque = 1,
  Transparent = 2,
  Overlay = 3,
};

struct SpherePrimitive {
  glm::vec3 center{0.0F, 0.0F, 0.0F};
  float radius = 0.1F;
  glm::vec4 color{1.0F, 1.0F, 1.0F, 1.0F};
  RenderPass pass = RenderPass::Opaque;
};

struct BoxPrimitive {
  glm::vec3 center{0.0F, 0.0F, 0.0F};
  glm::vec3 size{1.0F, 1.0F, 1.0F};
  float yawDeg = 0.0F;
  glm::vec4 color{1.0F, 1.0F, 1.0F, 1.0F};
  RenderPass pass = RenderPass::Opaque;
};

struct LinePrimitive {
  glm::vec3 a{0.0F, 0.0F, 0.0F};
  glm::vec3 b{0.0F, 0.0F, 0.0F};
  glm::vec4 color{1.0F, 1.0F, 1.0F, 1.0F};
  float width = 1.0F;
  RenderPass pass = RenderPass::Transparent;
};

struct MeshInstancePrimitive {
  std::string assetPath;
  glm::vec3 position{0.0F, 0.0F, 0.0F};
  glm::vec3 rotationDeg{0.0F, 0.0F, 0.0F};
  glm::vec3 scale{1.0F, 1.0F, 1.0F};
  glm::vec4 color{1.0F, 1.0F, 1.0F, 1.0F};
  bool useAssetColor = false;
  bool centerOnMeshBounds = false;
  bool wireframe = false;
  RenderPass pass = RenderPass::Opaque;
};

enum class OverlayAnchor {
  TopLeft,
  TopRight,
  BottomLeft,
  BottomRight,
};

struct OverlayLine {
  std::string text;
  glm::vec4 color{0.92F, 0.92F, 0.92F, 0.90F};
  OverlayAnchor anchor = OverlayAnchor::TopLeft;
  float marginX = 10.0F;
  float marginY = 12.0F;
};

using RenderEntityPayload = std::variant<SpherePrimitive, BoxPrimitive, LinePrimitive, MeshInstancePrimitive, OverlayLine>;

struct Transform3D {
  glm::vec3 position{0.0F, 0.0F, 0.0F};
  glm::vec3 rotationDeg{0.0F, 0.0F, 0.0F};
  glm::vec3 scale{1.0F, 1.0F, 1.0F};
};

struct EntityCulling {
  bool enabled = true;
  float boundsRadius = 0.5F;
};

struct RenderEntity {
  std::string id;
  RenderPass pass = RenderPass::Opaque;
  RenderEntityPayload payload = SpherePrimitive{};
  std::string parentId;
  Transform3D transform;
  bool hasTransform = false;
  EntityCulling culling;
};

struct RenderSceneFrame {
  bool drawFieldImage = true;
  bool drawGrid = true;
  bool drawAxes = true;

  std::vector<SpherePrimitive> spheres;
  std::vector<BoxPrimitive> boxes;
  std::vector<LinePrimitive> lines;
  std::vector<MeshInstancePrimitive> meshInstances;
  std::vector<OverlayLine> overlayLines;
  std::vector<RenderEntity> entities;
};

}  // namespace repulsor3d
