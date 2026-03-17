#pragma once

#include <glm/vec4.hpp>

namespace repulsor3d {

class IRenderBackend {
 public:
  virtual ~IRenderBackend() = default;

  virtual void ConfigureDefaultState() = 0;
  virtual void ClearFrame(const glm::vec4& clearColor) = 0;
  virtual void ResizeViewport(int width, int height) = 0;
  virtual void SetDepthTestEnabled(bool enabled) = 0;
};

class OpenGLRenderBackend final : public IRenderBackend {
 public:
  void ConfigureDefaultState() override;
  void ClearFrame(const glm::vec4& clearColor) override;
  void ResizeViewport(int width, int height) override;
  void SetDepthTestEnabled(bool enabled) override;
};

}  // namespace repulsor3d
