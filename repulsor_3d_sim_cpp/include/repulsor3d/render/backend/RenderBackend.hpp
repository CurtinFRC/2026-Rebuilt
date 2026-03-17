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
  virtual int GetUniformLocation(unsigned int programId, const char* uniformName) = 0;
  virtual void SetUniformMat4(int location, const float* matrix4x4) = 0;
  virtual void SetUniformVec4(int location, float x, float y, float z, float w) = 0;
  virtual void SetUniformVec3(int location, float x, float y, float z) = 0;
  virtual void SetUniform1f(int location, float value) = 0;
  virtual void SetUniform1i(int location, int value) = 0;
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
  int GetUniformLocation(unsigned int programId, const char* uniformName) override;
  void SetUniformMat4(int location, const float* matrix4x4) override;
  void SetUniformVec4(int location, float x, float y, float z, float w) override;
  void SetUniformVec3(int location, float x, float y, float z) override;
  void SetUniform1f(int location, float value) override;
  void SetUniform1i(int location, int value) override;
  void BindVertexArray(unsigned int vaoId) override;
  void BindArrayBuffer(unsigned int bufferId) override;
  void BindTexture2D(unsigned int textureId) override;
  void DrawTriangles(int vertexCount) override;
  void DrawIndexedTriangles(int indexCount) override;
  void DrawLines(int vertexCount, float lineWidth) override;
};

}  // namespace repulsor3d
