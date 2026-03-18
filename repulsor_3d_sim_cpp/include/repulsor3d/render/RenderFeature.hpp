#pragma once

#include <memory>
#include <string>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include "repulsor3d/Diagnostics.hpp"
#include "repulsor3d/render/RenderCommandBuffer.hpp"
#include "repulsor3d/render/SceneFrame.hpp"

namespace repulsor3d {

class Renderer;

struct RenderFeatureContext {
  const glm::mat4& viewProjection;
  glm::vec3 cameraWorldPosition{0.0F, 0.0F, 0.0F};
  int viewportWidth = 1;
  int viewportHeight = 1;
  const RenderSceneFrame& frame;
  const RenderCommandBuffer& commandBuffer;
  const DiagnosticsSnapshot* diagnostics = nullptr;
};

struct RendererDrawApi {
  explicit RendererDrawApi(Renderer& renderer);

  void DrawGrid(const glm::mat4& viewProjection) const;
  void DrawAxes(const glm::mat4& viewProjection) const;
  void DrawFieldImage(const glm::mat4& viewProjection) const;

  void DrawSphere(const glm::mat4& viewProjection, const SpherePrimitive& primitive) const;
  void DrawBox(const glm::mat4& viewProjection, const BoxPrimitive& primitive) const;
  void DrawLines(const glm::mat4& viewProjection, const std::vector<LinePrimitive>& lines) const;

  void DrawOverlay(int width, int height, const std::vector<OverlayLine>& lines) const;

 private:
  Renderer& renderer_;
};

class IRenderFeature {
 public:
  virtual ~IRenderFeature() = default;

  virtual std::string Name() const = 0;
  virtual std::vector<std::string> Dependencies() const { return {}; }

  virtual bool Initialize(Renderer& /*renderer*/) { return true; }
  virtual void Render(const RenderFeatureContext& context, const RendererDrawApi& drawApi) = 0;
};

std::vector<std::unique_ptr<IRenderFeature>> CreateDefaultRenderFeatures();

}  // namespace repulsor3d
