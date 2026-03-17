#include "repulsor3d/render/RenderCommandBuffer.hpp"

namespace repulsor3d {

RenderCommandBuffer BuildRenderCommandBuffer(const RenderSceneFrame& frame) {
  RenderCommandBuffer commands;
  commands.reserve(
      3 +
      frame.spheres.size() +
      frame.boxes.size() +
      frame.lines.size() +
      frame.meshInstances.size() +
      frame.overlayLines.size());

  commands.push_back(DrawGridCommand{frame.drawGrid});
  commands.push_back(DrawFieldImageCommand{frame.drawFieldImage});
  commands.push_back(DrawAxesCommand{frame.drawAxes});

  for (const auto& sphere : frame.spheres) {
    commands.push_back(DrawSphereCommand{sphere});
  }
  for (const auto& box : frame.boxes) {
    commands.push_back(DrawBoxCommand{box});
  }
  for (const auto& line : frame.lines) {
    commands.push_back(DrawLineCommand{line});
  }
  for (const auto& mesh : frame.meshInstances) {
    commands.push_back(DrawMeshInstanceCommand{mesh});
  }
  for (const auto& overlay : frame.overlayLines) {
    commands.push_back(DrawOverlayCommand{overlay});
  }

  return commands;
}

}  // namespace repulsor3d
