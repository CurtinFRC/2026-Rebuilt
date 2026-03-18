#include "repulsor3d/render/RenderFeature.hpp"

#include <memory>
#include <string>
#include <type_traits>
#include <utility>
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

class GeometryPassRenderFeature final : public IRenderFeature {
 public:
  GeometryPassRenderFeature(RenderPass renderPass, std::string featureName, std::vector<std::string> dependencies)
      : renderPass_(renderPass), featureName_(std::move(featureName)), dependencies_(std::move(dependencies)) {}

  std::string Name() const override { return featureName_; }
  std::vector<std::string> Dependencies() const override { return dependencies_; }

  void Render(const RenderFeatureContext& context, const RendererDrawApi& drawApi) override {
    std::vector<LinePrimitive> lines;
    for (const auto& command : context.commandBuffer) {
      std::visit(
          [&](const auto& typed) {
            using T = std::decay_t<decltype(typed)>;
            if constexpr (std::is_same_v<T, DrawSphereCommand>) {
              if (typed.pass == renderPass_) {
                drawApi.DrawSphere(context.viewProjection, typed.primitive);
              }
            } else if constexpr (std::is_same_v<T, DrawBoxCommand>) {
              if (typed.pass == renderPass_) {
                drawApi.DrawBox(context.viewProjection, typed.primitive);
              }
            } else if constexpr (std::is_same_v<T, DrawLineCommand>) {
              if (typed.pass == renderPass_) {
                lines.push_back(typed.primitive);
              }
            }
          },
          command);
    }
    drawApi.DrawLines(context.viewProjection, lines);
  }

 private:
  RenderPass renderPass_ = RenderPass::Opaque;
  std::string featureName_;
  std::vector<std::string> dependencies_;
};

class OverlayRenderFeature final : public IRenderFeature {
 public:
  std::string Name() const override { return "overlay"; }
  std::vector<std::string> Dependencies() const override { return {"cad_transparent"}; }

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

    if (context.diagnostics != nullptr && context.showDebugPanel) {
      lines.push_back(
          {"Frame ms: " + std::to_string(context.diagnostics->frameMilliseconds) +
           " (avg " + std::to_string(context.diagnostics->frameAverageMilliseconds) + ")"});
      lines.push_back({"Debug sections [1]Counters [2]CPU [3]GPU [4]Assets"});
      if (context.showDebugCounters) {
        for (const auto& counter : context.diagnostics->counters) {
          lines.push_back(
              {"[Counter " + counter.name + "] " + std::to_string(counter.value) +
               " (avg " + std::to_string(counter.averageValue) + ")"});
        }
      }
      if (context.showDebugCpu) {
        for (const auto& pass : context.diagnostics->passTimings) {
          lines.push_back(
              {"[CPU " + pass.name + "] " + std::to_string(pass.milliseconds) +
               " ms (avg " + std::to_string(pass.averageMilliseconds) + ")"});
        }
      }
      if (context.showDebugGpu) {
        for (const auto& gpu : context.diagnostics->gpuTimings) {
          lines.push_back(
              {"[GPU " + gpu.name + "] " + std::to_string(gpu.milliseconds) +
               " ms (avg " + std::to_string(gpu.averageMilliseconds) + ")"});
        }
      }
      if (context.showDebugAssets) {
        for (const auto& asset : context.diagnostics->assetTimings) {
          lines.push_back(
              {"[Asset " + asset.name + "] " + std::to_string(asset.milliseconds) +
               " ms (avg " + std::to_string(asset.averageMilliseconds) + ")"});
        }
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
  features.push_back(
      std::make_unique<GeometryPassRenderFeature>(RenderPass::Opaque, "geometry_opaque", std::vector<std::string>{"world"}));
  features.push_back(std::make_unique<CadModelRenderFeature>(
      RenderPass::Opaque, "cad_opaque", std::vector<std::string>{"geometry_opaque"}));
  features.push_back(std::make_unique<GeometryPassRenderFeature>(
      RenderPass::Transparent, "geometry_transparent", std::vector<std::string>{"cad_opaque"}));
  features.push_back(std::make_unique<CadModelRenderFeature>(
      RenderPass::Transparent, "cad_transparent", std::vector<std::string>{"geometry_transparent"}));
  features.push_back(std::make_unique<OverlayRenderFeature>());
  return features;
}

}  // namespace repulsor3d
