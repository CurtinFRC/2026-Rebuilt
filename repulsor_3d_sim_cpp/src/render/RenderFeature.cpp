#include "repulsor3d/render/RenderFeature.hpp"

#include <memory>
#include <vector>

#include "repulsor3d/Renderer.hpp"
#include "repulsor3d/render/CadModelRenderFeature.hpp"

namespace repulsor3d {
namespace {

class WorldRenderFeature final : public IRenderFeature {
 public:
  void Render(const RenderFeatureContext& context, const RendererDrawApi& drawApi) override {
    if (context.frame.drawGrid) {
      drawApi.DrawGrid(context.viewProjection);
    }
    if (context.frame.drawFieldImage) {
      drawApi.DrawFieldImage(context.viewProjection);
    }
    if (context.frame.drawAxes) {
      drawApi.DrawAxes(context.viewProjection);
    }
  }
};

class PrimitiveRenderFeature final : public IRenderFeature {
 public:
  void Render(const RenderFeatureContext& context, const RendererDrawApi& drawApi) override {
    for (const auto& sphere : context.frame.spheres) {
      drawApi.DrawSphere(context.viewProjection, sphere);
    }

    for (const auto& box : context.frame.boxes) {
      drawApi.DrawBox(context.viewProjection, box);
    }

    drawApi.DrawLines(context.viewProjection, context.frame.lines);
  }
};

class OverlayRenderFeature final : public IRenderFeature {
 public:
  void Render(const RenderFeatureContext& context, const RendererDrawApi& drawApi) override {
    drawApi.DrawOverlay(context.viewportWidth, context.viewportHeight, context.frame.overlayLines);
  }
};

}  // namespace

RendererDrawApi::RendererDrawApi(Renderer& renderer) : renderer_(renderer) {}

void RendererDrawApi::DrawGrid(const glm::mat4& viewProjection) const {
  renderer_.DrawGrid(viewProjection);
}

void RendererDrawApi::DrawAxes(const glm::mat4& viewProjection) const {
  renderer_.DrawAxes(viewProjection);
}

void RendererDrawApi::DrawFieldImage(const glm::mat4& viewProjection) const {
  renderer_.DrawFieldImage(viewProjection);
}

void RendererDrawApi::DrawSphere(const glm::mat4& viewProjection, const SpherePrimitive& primitive) const {
  renderer_.DrawSphere(viewProjection, primitive);
}

void RendererDrawApi::DrawBox(const glm::mat4& viewProjection, const BoxPrimitive& primitive) const {
  renderer_.DrawBox(viewProjection, primitive);
}

void RendererDrawApi::DrawLines(const glm::mat4& viewProjection, const std::vector<LinePrimitive>& lines) const {
  renderer_.DrawLinePrimitives(viewProjection, lines);
}

void RendererDrawApi::DrawOverlay(const int width, const int height, const std::vector<OverlayLine>& lines) const {
  renderer_.DrawOverlay(width, height, lines);
}

std::vector<std::unique_ptr<IRenderFeature>> CreateDefaultRenderFeatures() {
  std::vector<std::unique_ptr<IRenderFeature>> features;
  features.push_back(std::make_unique<WorldRenderFeature>());
  features.push_back(std::make_unique<PrimitiveRenderFeature>());
  features.push_back(std::make_unique<CadModelRenderFeature>());
  features.push_back(std::make_unique<OverlayRenderFeature>());
  return features;
}

}  // namespace repulsor3d
