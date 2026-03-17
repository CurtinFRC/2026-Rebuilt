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

void OpenGLRenderBackend::BindVertexArray(const unsigned int vaoId) {
  glBindVertexArray(vaoId);
}

void OpenGLRenderBackend::BindArrayBuffer(const unsigned int bufferId) {
  glBindBuffer(GL_ARRAY_BUFFER, bufferId);
}

void OpenGLRenderBackend::BindTexture2D(const unsigned int textureId) {
  glBindTexture(GL_TEXTURE_2D, textureId);
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

}  // namespace repulsor3d
