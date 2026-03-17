#pragma once

#include <variant>
#include <vector>

#include "repulsor3d/render/SceneFrame.hpp"

namespace repulsor3d {

struct DrawGridCommand {
  bool enabled = true;
};

struct DrawFieldImageCommand {
  bool enabled = true;
};

struct DrawAxesCommand {
  bool enabled = true;
};

struct DrawSphereCommand {
  SpherePrimitive primitive;
  RenderPass pass = RenderPass::Opaque;
};

struct DrawBoxCommand {
  BoxPrimitive primitive;
  RenderPass pass = RenderPass::Opaque;
};

struct DrawLineCommand {
  LinePrimitive primitive;
  RenderPass pass = RenderPass::Transparent;
};

struct DrawMeshInstanceCommand {
  MeshInstancePrimitive primitive;
  RenderPass pass = RenderPass::Opaque;
};

struct DrawOverlayCommand {
  OverlayLine line;
  RenderPass pass = RenderPass::Overlay;
};

using RenderCommand = std::variant<
    DrawGridCommand,
    DrawFieldImageCommand,
    DrawAxesCommand,
    DrawSphereCommand,
    DrawBoxCommand,
    DrawLineCommand,
    DrawMeshInstanceCommand,
    DrawOverlayCommand>;

using RenderCommandBuffer = std::vector<RenderCommand>;

RenderCommandBuffer BuildRenderCommandBuffer(const RenderSceneFrame& frame);

}  // namespace repulsor3d
