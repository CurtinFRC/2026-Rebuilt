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

}  // namespace repulsor3d
