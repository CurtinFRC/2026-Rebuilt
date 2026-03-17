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
  virtual void SetWireframeMode(bool enabled) = 0;
  virtual void UseProgram(unsigned int programId) = 0;
  virtual void BindVertexArray(unsigned int vaoId) = 0;
  virtual void BindArrayBuffer(unsigned int bufferId) = 0;
  virtual void BindTexture2D(unsigned int textureId) = 0;
  virtual void DrawTriangles(int vertexCount) = 0;
  virtual void DrawIndexedTriangles(int indexCount) = 0;
  virtual void DrawLines(int vertexCount, float lineWidth) = 0;
};

class OpenGLRenderBackend final : public IRenderBackend {
 public:
  void ConfigureDefaultState() override;
  void ClearFrame(const glm::vec4& clearColor) override;
  void ResizeViewport(int width, int height) override;
  void SetDepthTestEnabled(bool enabled) override;
  void SetWireframeMode(bool enabled) override;
  void UseProgram(unsigned int programId) override;
  void BindVertexArray(unsigned int vaoId) override;
  void BindArrayBuffer(unsigned int bufferId) override;
  void BindTexture2D(unsigned int textureId) override;
  void DrawTriangles(int vertexCount) override;
  void DrawIndexedTriangles(int indexCount) override;
  void DrawLines(int vertexCount, float lineWidth) override;
};

}  // namespace repulsor3d
