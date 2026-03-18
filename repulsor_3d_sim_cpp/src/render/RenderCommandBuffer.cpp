#include "repulsor3d/render/RenderCommandBuffer.hpp"

#include <array>
#include <type_traits>
#include <variant>

namespace repulsor3d {
namespace {

size_t PassIndex(const RenderPass pass) {
  switch (pass) {
    case RenderPass::Background:
      return 0;
    case RenderPass::Opaque:
      return 1;
    case RenderPass::Transparent:
      return 2;
    case RenderPass::Overlay:
      return 3;
  }
  return 1;
}

void AddEntityCommand(const RenderEntity& entity, std::array<RenderCommandBuffer, 4>& passBuckets) {
  std::visit(
      [&](const auto& payload) {
        using T = std::decay_t<decltype(payload)>;
        if constexpr (std::is_same_v<T, SpherePrimitive>) {
          SpherePrimitive primitive = payload;
          primitive.pass = entity.pass;
          passBuckets[PassIndex(entity.pass)].push_back(DrawSphereCommand{primitive, entity.pass});
        } else if constexpr (std::is_same_v<T, BoxPrimitive>) {
          BoxPrimitive primitive = payload;
          primitive.pass = entity.pass;
          passBuckets[PassIndex(entity.pass)].push_back(DrawBoxCommand{primitive, entity.pass});
        } else if constexpr (std::is_same_v<T, LinePrimitive>) {
          LinePrimitive primitive = payload;
          primitive.pass = entity.pass;
          passBuckets[PassIndex(entity.pass)].push_back(DrawLineCommand{primitive, entity.pass});
        } else if constexpr (std::is_same_v<T, MeshInstancePrimitive>) {
          MeshInstancePrimitive primitive = payload;
          primitive.pass = entity.pass;
          passBuckets[PassIndex(entity.pass)].push_back(DrawMeshInstanceCommand{primitive, entity.pass});
        } else if constexpr (std::is_same_v<T, OverlayLine>) {
          passBuckets[PassIndex(RenderPass::Overlay)].push_back(DrawOverlayCommand{payload, RenderPass::Overlay});
        }
      },
      entity.payload);
}

RenderEntity ToEntity(const SceneNode& node) {
  return RenderEntity{
      .id = node.id,
      .pass = node.pass,
      .payload = node.payload,
      .parentId = node.parentId,
      .transform = node.localTransform,
      .hasTransform = node.hasLocalTransform,
      .culling = node.culling,
  };
}

}  // namespace

RenderCommandBuffer BuildRenderCommandBuffer(const RenderSceneFrame& frame) {
  RenderSceneFrame expanded = frame;
  for (const auto& binding : expanded.components) {
    if (binding.component == nullptr) {
      continue;
    }
    binding.component->AppendToFrame(expanded);
  }

  RenderCommandBuffer commands;
  commands.reserve(
      3 +
      expanded.spheres.size() +
      expanded.boxes.size() +
      expanded.lines.size() +
      expanded.meshInstances.size() +
      expanded.overlayLines.size() +
      expanded.entities.size() +
      expanded.sceneNodes.size());

  std::array<RenderCommandBuffer, 4> passBuckets;
  commands.push_back(DrawGridCommand{expanded.drawGrid});
  commands.push_back(DrawFieldImageCommand{expanded.drawFieldImage});
  commands.push_back(DrawAxesCommand{expanded.drawAxes});

  for (const auto& sphere : expanded.spheres) {
    passBuckets[PassIndex(sphere.pass)].push_back(DrawSphereCommand{sphere, sphere.pass});
  }
  for (const auto& box : expanded.boxes) {
    passBuckets[PassIndex(box.pass)].push_back(DrawBoxCommand{box, box.pass});
  }
  for (const auto& line : expanded.lines) {
    passBuckets[PassIndex(line.pass)].push_back(DrawLineCommand{line, line.pass});
  }
  for (const auto& mesh : expanded.meshInstances) {
    passBuckets[PassIndex(mesh.pass)].push_back(DrawMeshInstanceCommand{mesh, mesh.pass});
  }
  for (const auto& overlay : expanded.overlayLines) {
    passBuckets[PassIndex(RenderPass::Overlay)].push_back(DrawOverlayCommand{overlay, RenderPass::Overlay});
  }
  for (const auto& entity : expanded.entities) {
    AddEntityCommand(entity, passBuckets);
  }
  for (const auto& node : expanded.sceneNodes) {
    AddEntityCommand(ToEntity(node), passBuckets);
  }

  for (auto& bucket : passBuckets) {
    commands.insert(commands.end(), bucket.begin(), bucket.end());
  }

  return commands;
}

}  // namespace repulsor3d
