#include "repulsor3d/render/backend/RenderBackend.hpp"

#include <GL/glew.h>

namespace repulsor3d {

void OpenGLRenderBackend::ConfigureDefaultState() {
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
}

void OpenGLRenderBackend::ClearFrame(const glm::vec4& clearColor) {
  glClearColor(clearColor.r, clearColor.g, clearColor.b, clearColor.a);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void OpenGLRenderBackend::ResizeViewport(const int width, const int height) {
  glViewport(0, 0, width, height);
}

void OpenGLRenderBackend::SetDepthTestEnabled(const bool enabled) {
  if (enabled) {
    glEnable(GL_DEPTH_TEST);
  } else {
    glDisable(GL_DEPTH_TEST);
  }
}

void OpenGLRenderBackend::SetWireframeMode(const bool enabled) {
  glPolygonMode(GL_FRONT_AND_BACK, enabled ? GL_LINE : GL_FILL);
}

void OpenGLRenderBackend::UseProgram(const unsigned int programId) {
  glUseProgram(programId);
}

int OpenGLRenderBackend::GetUniformLocation(const unsigned int programId, const char* uniformName) {
  return glGetUniformLocation(programId, uniformName);
}

void OpenGLRenderBackend::SetUniformMat4(const int location, const float* matrix4x4) {
  glUniformMatrix4fv(location, 1, GL_FALSE, matrix4x4);
}

void OpenGLRenderBackend::SetUniformVec4(const int location, const float x, const float y, const float z, const float w) {
  glUniform4f(location, x, y, z, w);
}

void OpenGLRenderBackend::SetUniformVec3(const int location, const float x, const float y, const float z) {
  glUniform3f(location, x, y, z);
}

void OpenGLRenderBackend::SetUniform1f(const int location, const float value) {
  glUniform1f(location, value);
}

void OpenGLRenderBackend::SetUniform1i(const int location, const int value) {
  glUniform1i(location, value);
}

unsigned int OpenGLRenderBackend::CreateVertexArray() {
  unsigned int vao = 0;
  glGenVertexArrays(1, &vao);
  return vao;
}

unsigned int OpenGLRenderBackend::CreateBuffer() {
  unsigned int buffer = 0;
  glGenBuffers(1, &buffer);
  return buffer;
}

unsigned int OpenGLRenderBackend::CreateTexture2D() {
  unsigned int texture = 0;
  glGenTextures(1, &texture);
  return texture;
}

void OpenGLRenderBackend::BindVertexArray(const unsigned int vaoId) {
  glBindVertexArray(vaoId);
}

void OpenGLRenderBackend::BindArrayBuffer(const unsigned int bufferId) {
  glBindBuffer(GL_ARRAY_BUFFER, bufferId);
}

void OpenGLRenderBackend::BindElementArrayBuffer(const unsigned int bufferId) {
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bufferId);
}

void OpenGLRenderBackend::UploadArrayBufferData(const std::size_t sizeBytes, const void* data, const bool dynamic) {
  glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(sizeBytes), data, dynamic ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW);
}

void OpenGLRenderBackend::UploadElementArrayBufferData(const std::size_t sizeBytes, const void* data, const bool dynamic) {
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(sizeBytes), data, dynamic ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW);
}

void OpenGLRenderBackend::EnableVertexAttrib(const unsigned int index) {
  glEnableVertexAttribArray(index);
}

void OpenGLRenderBackend::DefineVertexAttribFloat(
    const unsigned int index,
    const int componentCount,
    const int strideBytes,
    const std::size_t offsetBytes) {
  glVertexAttribPointer(
      index,
      componentCount,
      GL_FLOAT,
      GL_FALSE,
      strideBytes,
      reinterpret_cast<void*>(offsetBytes));
}

void OpenGLRenderBackend::BindTexture2D(const unsigned int textureId) {
  glBindTexture(GL_TEXTURE_2D, textureId);
}

void OpenGLRenderBackend::SetTexture2DLinearMipmapClamp() {
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

void OpenGLRenderBackend::UploadTexture2DRgba8(const int width, const int height, const void* pixels) {
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
}

void OpenGLRenderBackend::GenerateTexture2DMipmaps() {
  glGenerateMipmap(GL_TEXTURE_2D);
}

void OpenGLRenderBackend::SetActiveTextureUnit(const int unit) {
  glActiveTexture(GL_TEXTURE0 + unit);
}

void OpenGLRenderBackend::DrawTriangles(const int vertexCount) {
  glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertexCount));
}

void OpenGLRenderBackend::DrawIndexedTriangles(const int indexCount) {
  glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr);
}

void OpenGLRenderBackend::DrawLines(const int vertexCount, const float lineWidth) {
  glLineWidth(lineWidth);
  glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertexCount));
}

unsigned int OpenGLRenderBackend::CreateGpuTimerQuery() {
  GLuint query = 0;
  glGenQueries(1, &query);
  return query;
}

void OpenGLRenderBackend::DestroyGpuTimerQuery(const unsigned int queryId) {
  if (queryId == 0) {
    return;
  }
  GLuint q = queryId;
  glDeleteQueries(1, &q);
}

void OpenGLRenderBackend::BeginGpuTimerQuery(const unsigned int queryId) {
  if (queryId == 0) {
    return;
  }
  glBeginQuery(GL_TIME_ELAPSED, queryId);
}

void OpenGLRenderBackend::EndGpuTimerQuery() {
  glEndQuery(GL_TIME_ELAPSED);
}

bool OpenGLRenderBackend::TryReadGpuTimerMilliseconds(const unsigned int queryId, double& outMilliseconds) {
  outMilliseconds = 0.0;
  if (queryId == 0) {
    return false;
  }

  GLint available = 0;
  glGetQueryObjectiv(queryId, GL_QUERY_RESULT_AVAILABLE, &available);
  if (available == 0) {
    return false;
  }

  GLuint64 elapsedNanoseconds = 0;
  glGetQueryObjectui64v(queryId, GL_QUERY_RESULT, &elapsedNanoseconds);
  outMilliseconds = static_cast<double>(elapsedNanoseconds) * 1e-6;
  return true;
}

}  // namespace repulsor3d
