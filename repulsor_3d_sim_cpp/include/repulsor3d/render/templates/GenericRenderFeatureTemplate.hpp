#pragma once

#include "repulsor3d/render/RenderFeature.hpp"

namespace repulsor3d {

// Copy this template to quickly create a new render module/pipeline stage.
class GenericRenderFeatureTemplate final : public IRenderFeature {
 public:
  bool Initialize(Renderer& /*renderer*/) override {
    // Allocate GPU resources here (meshes, shaders, textures).
    return true;
  }

  void Render(const RenderFeatureContext& context, const RendererDrawApi& drawApi) override {
    // Example usage:
    // if (context.frame.drawGrid) { drawApi.DrawGrid(context.viewProjection); }
    // drawApi.DrawOverlay(context.viewportWidth, context.viewportHeight, context.frame.overlayLines);
    (void)context;
    (void)drawApi;
  }
};

}  // namespace repulsor3d
