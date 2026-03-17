#include "repulsor3d/render/RenderFeature.hpp"

#include <memory>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "repulsor3d/Renderer.hpp"
#include "repulsor3d/render/CadModelRenderFeature.hpp"

namespace repulsor3d {
namespace {

class WorldRenderFeature final : public IRenderFeature {
 public:
  std::string Name() const override { return "world"; }

  void Render(const RenderFeatureContext& context, const RendererDrawApi& drawApi) override {
    for (const auto& command : context.commandBuffer) {
      std::visit(
          [&](const auto& typed) {
            using T = std::decay_t<decltype(typed)>;
            if constexpr (std::is_same_v<T, DrawGridCommand>) {
              if (typed.enabled) {
                drawApi.DrawGrid(context.viewProjection);
              }
            } else if constexpr (std::is_same_v<T, DrawFieldImageCommand>) {
              if (typed.enabled) {
                drawApi.DrawFieldImage(context.viewProjection);
              }
            } else if constexpr (std::is_same_v<T, DrawAxesCommand>) {
              if (typed.enabled) {
                drawApi.DrawAxes(context.viewProjection);
              }
            }
          },
          command);
    }
  }
};

class PrimitiveRenderFeature final : public IRenderFeature {
 public:
  std::string Name() const override { return "primitives"; }
  std::vector<std::string> Dependencies() const override { return {"world"}; }

  void Render(const RenderFeatureContext& context, const RendererDrawApi& drawApi) override {
    std::vector<LinePrimitive> lines;
    for (const auto& command : context.commandBuffer) {
      std::visit(
          [&](const auto& typed) {
            using T = std::decay_t<decltype(typed)>;
            if constexpr (std::is_same_v<T, DrawSphereCommand>) {
              drawApi.DrawSphere(context.viewProjection, typed.primitive);
            } else if constexpr (std::is_same_v<T, DrawBoxCommand>) {
              drawApi.DrawBox(context.viewProjection, typed.primitive);
            } else if constexpr (std::is_same_v<T, DrawLineCommand>) {
              lines.push_back(typed.primitive);
            }
          },
          command);
    }
    drawApi.DrawLines(context.viewProjection, lines);
  }
};

class OverlayRenderFeature final : public IRenderFeature {
 public:
  std::string Name() const override { return "overlay"; }
  std::vector<std::string> Dependencies() const override { return {"primitives", "cad_models"}; }

  void Render(const RenderFeatureContext& context, const RendererDrawApi& drawApi) override {
    std::vector<OverlayLine> lines;
    lines.reserve(context.frame.overlayLines.size() + 8);

    for (const auto& command : context.commandBuffer) {
      std::visit(
          [&](const auto& typed) {
            using T = std::decay_t<decltype(typed)>;
            if constexpr (std::is_same_v<T, DrawOverlayCommand>) {
              lines.push_back(typed.line);
            }
          },
          command);
    }

    if (context.diagnostics != nullptr) {
      lines.push_back({"Frame ms: " + std::to_string(context.diagnostics->frameMilliseconds)});
      for (const auto& feature : context.diagnostics->featureTimings) {
        lines.push_back({"[" + feature.name + "] " + std::to_string(feature.milliseconds) + " ms"});
      }
    }

    drawApi.DrawOverlay(context.viewportWidth, context.viewportHeight, lines);
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
