#pragma once

#include <cstddef>
#include <string>

#include <glm/vec4.hpp>

namespace repulsor3d {

struct RenderBackendCapabilities {
  std::string backendName = "unknown";
  int maxTextureSize = 0;
  int maxVertexAttribs = 0;
  bool supportsGpuTimers = false;
  bool supportsInstancing = false;
  bool supportsMipmapTextures = false;
};

class IRenderBackend {
 public:
  virtual ~IRenderBackend() = default;

  virtual const RenderBackendCapabilities& Capabilities() const = 0;

  virtual void ConfigureDefaultState() = 0;
  virtual void ClearFrame(const glm::vec4& clearColor) = 0;
  virtual void ResizeViewport(int width, int height) = 0;
  virtual void SetDepthTestEnabled(bool enabled) = 0;
  virtual void SetBlendEnabled(bool enabled) = 0;
  virtual void SetWireframeMode(bool enabled) = 0;
  virtual void UseProgram(unsigned int programId) = 0;
  virtual int GetUniformLocation(unsigned int programId, const char* uniformName) = 0;
  virtual void SetUniformMat4(int location, const float* matrix4x4) = 0;
  virtual void SetUniformMat3(int location, const float* matrix3x3) = 0;
  virtual void SetUniformVec4(int location, float x, float y, float z, float w) = 0;
  virtual void SetUniformVec3(int location, float x, float y, float z) = 0;
  virtual void SetUniform1f(int location, float value) = 0;
  virtual void SetUniform1i(int location, int value) = 0;
  virtual unsigned int CreateVertexArray() = 0;
  virtual unsigned int CreateBuffer() = 0;
  virtual unsigned int CreateTexture2D() = 0;
  virtual void BindVertexArray(unsigned int vaoId) = 0;
  virtual void BindArrayBuffer(unsigned int bufferId) = 0;
  virtual void BindElementArrayBuffer(unsigned int bufferId) = 0;
  virtual void UploadArrayBufferData(std::size_t sizeBytes, const void* data, bool dynamic) = 0;
  virtual void UploadElementArrayBufferData(std::size_t sizeBytes, const void* data, bool dynamic) = 0;
  virtual void EnableVertexAttrib(unsigned int index) = 0;
  virtual void DefineVertexAttribFloat(unsigned int index, int componentCount, int strideBytes, std::size_t offsetBytes) = 0;
  virtual void DefineVertexAttribNormalizedU8(
      unsigned int index,
      int componentCount,
      int strideBytes,
      std::size_t offsetBytes) = 0;
  virtual void SetVertexAttribDivisor(unsigned int index, unsigned int divisor) = 0;
  virtual void BindTexture2D(unsigned int textureId) = 0;
  virtual void SetTexture2DLinearMipmapClamp() = 0;
  virtual void UploadTexture2DRgba8(int width, int height, const void* pixels) = 0;
  virtual void GenerateTexture2DMipmaps() = 0;
  virtual void SetActiveTextureUnit(int unit) = 0;
  virtual void DrawTriangles(int vertexCount) = 0;
  virtual void DrawIndexedTriangles(int indexCount) = 0;
  virtual void DrawTrianglesInstanced(int vertexCount, int instanceCount) = 0;
  virtual void DrawIndexedTrianglesInstanced(int indexCount, int instanceCount) = 0;
  virtual void DrawLines(int vertexCount, float lineWidth) = 0;

  virtual unsigned int CreateGpuTimerQuery() = 0;
  virtual void DestroyGpuTimerQuery(unsigned int queryId) = 0;
  virtual void BeginGpuTimerQuery(unsigned int queryId) = 0;
  virtual void EndGpuTimerQuery() = 0;
  virtual bool TryReadGpuTimerMilliseconds(unsigned int queryId, double& outMilliseconds) = 0;
};

class OpenGLRenderBackend final : public IRenderBackend {
 public:
  const RenderBackendCapabilities& Capabilities() const override { return capabilities_; }
  void ConfigureDefaultState() override;
  void ClearFrame(const glm::vec4& clearColor) override;
  void ResizeViewport(int width, int height) override;
  void SetDepthTestEnabled(bool enabled) override;
  void SetBlendEnabled(bool enabled) override;
  void SetWireframeMode(bool enabled) override;
  void UseProgram(unsigned int programId) override;
  int GetUniformLocation(unsigned int programId, const char* uniformName) override;
  void SetUniformMat4(int location, const float* matrix4x4) override;
  void SetUniformMat3(int location, const float* matrix3x3) override;
  void SetUniformVec4(int location, float x, float y, float z, float w) override;
  void SetUniformVec3(int location, float x, float y, float z) override;
  void SetUniform1f(int location, float value) override;
  void SetUniform1i(int location, int value) override;
  unsigned int CreateVertexArray() override;
  unsigned int CreateBuffer() override;
  unsigned int CreateTexture2D() override;
  void BindVertexArray(unsigned int vaoId) override;
  void BindArrayBuffer(unsigned int bufferId) override;
  void BindElementArrayBuffer(unsigned int bufferId) override;
  void UploadArrayBufferData(std::size_t sizeBytes, const void* data, bool dynamic) override;
  void UploadElementArrayBufferData(std::size_t sizeBytes, const void* data, bool dynamic) override;
  void EnableVertexAttrib(unsigned int index) override;
  void DefineVertexAttribFloat(unsigned int index, int componentCount, int strideBytes, std::size_t offsetBytes) override;
  void DefineVertexAttribNormalizedU8(
      unsigned int index,
      int componentCount,
      int strideBytes,
      std::size_t offsetBytes) override;
  void SetVertexAttribDivisor(unsigned int index, unsigned int divisor) override;
  void BindTexture2D(unsigned int textureId) override;
  void SetTexture2DLinearMipmapClamp() override;
  void UploadTexture2DRgba8(int width, int height, const void* pixels) override;
  void GenerateTexture2DMipmaps() override;
  void SetActiveTextureUnit(int unit) override;
  void DrawTriangles(int vertexCount) override;
  void DrawIndexedTriangles(int indexCount) override;
  void DrawTrianglesInstanced(int vertexCount, int instanceCount) override;
  void DrawIndexedTrianglesInstanced(int indexCount, int instanceCount) override;
  void DrawLines(int vertexCount, float lineWidth) override;

  unsigned int CreateGpuTimerQuery() override;
  void DestroyGpuTimerQuery(unsigned int queryId) override;
  void BeginGpuTimerQuery(unsigned int queryId) override;
  void EndGpuTimerQuery() override;
  bool TryReadGpuTimerMilliseconds(unsigned int queryId, double& outMilliseconds) override;

 private:
  RenderBackendCapabilities capabilities_;
};

}  // namespace repulsor3d
