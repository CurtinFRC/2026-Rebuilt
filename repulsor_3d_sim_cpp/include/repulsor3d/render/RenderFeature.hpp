#pragma once

#include <memory>
#include <vector>

#include <glm/mat4x4.hpp>

#include "repulsor3d/render/SceneFrame.hpp"

namespace repulsor3d {

class Renderer;

struct RenderFeatureContext {
  const glm::mat4& viewProjection;
  int viewportWidth = 1;
  int viewportHeight = 1;
  const RenderSceneFrame& frame;
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

  virtual bool Initialize(Renderer& /*renderer*/) { return true; }
  virtual void Render(const RenderFeatureContext& context, const RendererDrawApi& drawApi) = 0;
};

std::vector<std::unique_ptr<IRenderFeature>> CreateDefaultRenderFeatures();

}  // namespace repulsor3d
