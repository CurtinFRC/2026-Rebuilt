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
};

struct DrawBoxCommand {
  BoxPrimitive primitive;
};

struct DrawLineCommand {
  LinePrimitive primitive;
};

struct DrawMeshInstanceCommand {
  MeshInstancePrimitive primitive;
};

struct DrawOverlayCommand {
  OverlayLine line;
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
