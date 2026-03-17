#include "repulsor3d/Renderer.hpp"

#include <GL/glew.h>

#include <algorithm>
#include <map>

#include <glm/common.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/trigonometric.hpp>

#include "repulsor3d/render/SceneModelBuilderFactory.hpp"

namespace repulsor3d {

Renderer::Renderer(const ViewerConfig& cfg, std::unique_ptr<ISceneModelBuilder> sceneBuilder)
    : cfg_(cfg), sceneBuilder_(std::move(sceneBuilder)) {
  showCameraDebug = cfg.showCameraDebug;
  showTruthFuel = cfg.showTruthFuel;
  showAgeFilteredFuel = cfg.showAgeFilteredFuel;
  showFieldImage = cfg.showFieldImage;

  width_ = std::max(1, cfg.windowW);
  height_ = std::max(1, cfg.windowH);

  fieldLength_ = cfg.fieldLengthM;
  fieldWidth_ = cfg.fieldWidthM;
  fieldZ_ = cfg.fieldZM;

  if (sceneBuilder_ == nullptr) {
    sceneBuilder_ = CreateDefaultSceneModelBuilder(cfg_);
  }
}

Renderer::~Renderer() {
  DestroyShader(solidShader_);
  DestroyShader(lineShader_);
  DestroyShader(texturedShader_);

  DestroyMesh(cubeMesh_);
  DestroyMesh(sphereMesh_);
  DestroyMesh(quadMesh_);

  if (dynamicLineVbo_ != 0) {
    glDeleteBuffers(1, &dynamicLineVbo_);
    dynamicLineVbo_ = 0;
  }
  if (dynamicLineVao_ != 0) {
    glDeleteVertexArrays(1, &dynamicLineVao_);
    dynamicLineVao_ = 0;
  }

  if (textVbo_ != 0) {
    glDeleteBuffers(1, &textVbo_);
    textVbo_ = 0;
  }
  if (textVao_ != 0) {
    glDeleteVertexArrays(1, &textVao_);
    textVao_ = 0;
  }

  if (fieldTexture_ != 0) {
    glDeleteTextures(1, &fieldTexture_);
    fieldTexture_ = 0;
  }
}

void Renderer::SetSceneModelBuilder(std::unique_ptr<ISceneModelBuilder> sceneBuilder) {
  if (sceneBuilder != nullptr) {
    sceneBuilder_ = std::move(sceneBuilder);
  }
}

bool Renderer::Initialize() {
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);

  glClearColor(0.06F, 0.06F, 0.07F, 1.0F);

  if (!CreateShaders()) {
    return false;
  }
  if (!CreateMeshes()) {
    return false;
  }
  if (!CreateFieldTexture()) {
    // Texture is optional.
  }

  glGenVertexArrays(1, &dynamicLineVao_);
  glGenBuffers(1, &dynamicLineVbo_);
  glBindVertexArray(dynamicLineVao_);
  glBindBuffer(GL_ARRAY_BUFFER, dynamicLineVbo_);
  glBufferData(GL_ARRAY_BUFFER, 1024 * sizeof(VertexPC), nullptr, GL_DYNAMIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPC), reinterpret_cast<void*>(offsetof(VertexPC, pos)));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(VertexPC), reinterpret_cast<void*>(offsetof(VertexPC, color)));
  glBindVertexArray(0);

  glGenVertexArrays(1, &textVao_);
  glGenBuffers(1, &textVbo_);
  glBindVertexArray(textVao_);
  glBindBuffer(GL_ARRAY_BUFFER, textVbo_);
  glBufferData(GL_ARRAY_BUFFER, 4096 * sizeof(glm::vec3), nullptr, GL_DYNAMIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), reinterpret_cast<void*>(0));
  glBindVertexArray(0);

  Resize(width_, height_);
  return true;
}

void Renderer::Resize(const int w, const int h) {
  width_ = std::max(1, w);
  height_ = std::max(1, h);
  glViewport(0, 0, width_, height_);
}

void Renderer::Draw(GLFWwindow* /*window*/, const OrbitCamera& camera, const SnapshotBundle& bundle) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  SceneToggleState toggles;
  toggles.showCameraDebug = showCameraDebug;
  toggles.showTruthFuel = showTruthFuel;
  toggles.showAgeFilteredFuel = showAgeFilteredFuel;
  toggles.showFieldImage = showFieldImage;

  RenderSceneFrame sceneFrame = sceneBuilder_->BuildFrame(bundle, toggles);

  const float aspect = static_cast<float>(width_) / static_cast<float>(height_);
  const glm::mat4 vp = camera.ProjectionMatrix(aspect) * camera.ViewMatrix();

  if (sceneFrame.drawGrid) {
    DrawGrid(vp);
  }
  if (sceneFrame.drawFieldImage && fieldTexture_ != 0) {
    DrawFieldImage(vp);
  }
  if (sceneFrame.drawAxes) {
    DrawAxes(vp);
  }

  for (const auto& sphere : sceneFrame.spheres) {
    DrawSphere(vp, sphere);
  }
  for (const auto& box : sceneFrame.boxes) {
    DrawBox(vp, box);
  }

  std::map<float, std::vector<VertexPC>> lineBatches;
  for (const auto& line : sceneFrame.lines) {
    auto& batch = lineBatches[line.width];
    batch.push_back({line.a, line.color});
    batch.push_back({line.b, line.color});
  }
  for (const auto& [width, verts] : lineBatches) {
    DrawLineList(vp, verts, width);
  }

  DrawOverlay(width_, height_, sceneFrame.overlayLines);
}

void Renderer::DrawGrid(const glm::mat4& vp) {
  std::vector<VertexPC> lines;
  lines.reserve(512);

  const glm::vec4 col{0.15F, 0.15F, 0.18F, 1.0F};
  const float step = 1.0F;

  for (float x = 0.0F; x <= fieldLength_ + 1e-4F; x += step) {
    lines.push_back({glm::vec3{x, 0.0F, fieldZ_}, col});
    lines.push_back({glm::vec3{x, fieldWidth_, fieldZ_}, col});
  }
  for (float y = 0.0F; y <= fieldWidth_ + 1e-4F; y += step) {
    lines.push_back({glm::vec3{0.0F, y, fieldZ_}, col});
    lines.push_back({glm::vec3{fieldLength_, y, fieldZ_}, col});
  }

  DrawLineList(vp, lines, 1.0F);
}

void Renderer::DrawAxes(const glm::mat4& vp) {
  std::vector<VertexPC> lines;
  lines.reserve(6);
  lines.push_back({glm::vec3{0.0F, 0.0F, fieldZ_}, glm::vec4{1.0F, 0.2F, 0.2F, 1.0F}});
  lines.push_back({glm::vec3{1.0F, 0.0F, fieldZ_}, glm::vec4{1.0F, 0.2F, 0.2F, 1.0F}});

  lines.push_back({glm::vec3{0.0F, 0.0F, fieldZ_}, glm::vec4{0.2F, 1.0F, 0.2F, 1.0F}});
  lines.push_back({glm::vec3{0.0F, 1.0F, fieldZ_}, glm::vec4{0.2F, 1.0F, 0.2F, 1.0F}});

  lines.push_back({glm::vec3{0.0F, 0.0F, fieldZ_}, glm::vec4{0.2F, 0.6F, 1.0F, 1.0F}});
  lines.push_back({glm::vec3{0.0F, 0.0F, fieldZ_ + 1.0F}, glm::vec4{0.2F, 0.6F, 1.0F, 1.0F}});

  DrawLineList(vp, lines, 2.0F);
}

void Renderer::DrawFieldImage(const glm::mat4& vp) {
  glUseProgram(texturedShader_.id);

  glm::mat4 model(1.0F);
  model = glm::translate(model, glm::vec3{0.0F, 0.0F, fieldZ_ + 0.001F});
  model = glm::scale(model, glm::vec3{fieldLength_, fieldWidth_, 1.0F});

  const glm::mat4 mvp = vp * model;
  glUniformMatrix4fv(glGetUniformLocation(texturedShader_.id, "uMvp"), 1, GL_FALSE, &mvp[0][0]);
  glUniform1f(glGetUniformLocation(texturedShader_.id, "uAlpha"), glm::clamp(cfg_.fieldImageAlpha, 0.0F, 1.0F));
  glUniform1i(glGetUniformLocation(texturedShader_.id, "uFlipX"), cfg_.fieldImageFlipX ? 1 : 0);
  glUniform1i(glGetUniformLocation(texturedShader_.id, "uFlipY"), cfg_.fieldImageFlipY ? 1 : 0);

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, fieldTexture_);
  glUniform1i(glGetUniformLocation(texturedShader_.id, "uTex"), 0);

  glBindVertexArray(quadMesh_.vao);
  glDrawElements(GL_TRIANGLES, quadMesh_.indexCount, GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
  glBindTexture(GL_TEXTURE_2D, 0);
}

void Renderer::DrawBox(const glm::mat4& vp, const BoxPrimitive& primitive) {
  glm::mat4 model(1.0F);
  model = glm::translate(model, primitive.center);
  model = glm::rotate(model, glm::radians(primitive.yawDeg), glm::vec3{0.0F, 0.0F, 1.0F});
  model = glm::scale(model, primitive.size);

  const glm::mat4 mvp = vp * model;

  glUseProgram(solidShader_.id);
  glUniformMatrix4fv(glGetUniformLocation(solidShader_.id, "uMvp"), 1, GL_FALSE, &mvp[0][0]);
  glUniform4f(glGetUniformLocation(solidShader_.id, "uColor"), primitive.color.r, primitive.color.g, primitive.color.b, primitive.color.a);

  glBindVertexArray(cubeMesh_.vao);
  glDrawElements(GL_TRIANGLES, cubeMesh_.indexCount, GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
}

void Renderer::DrawSphere(const glm::mat4& vp, const SpherePrimitive& primitive) {
  glm::mat4 model(1.0F);
  model = glm::translate(model, primitive.center);
  model = glm::scale(model, glm::vec3{primitive.radius, primitive.radius, primitive.radius});

  const glm::mat4 mvp = vp * model;

  glUseProgram(solidShader_.id);
  glUniformMatrix4fv(glGetUniformLocation(solidShader_.id, "uMvp"), 1, GL_FALSE, &mvp[0][0]);
  glUniform4f(glGetUniformLocation(solidShader_.id, "uColor"), primitive.color.r, primitive.color.g, primitive.color.b, primitive.color.a);

  glBindVertexArray(sphereMesh_.vao);
  glDrawElements(GL_TRIANGLES, sphereMesh_.indexCount, GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
}

void Renderer::DrawLineList(const glm::mat4& vp, const std::vector<VertexPC>& vertices, const float width) {
  if (vertices.empty()) {
    return;
  }

  glUseProgram(lineShader_.id);
  glUniformMatrix4fv(glGetUniformLocation(lineShader_.id, "uMvp"), 1, GL_FALSE, &vp[0][0]);

  glBindVertexArray(dynamicLineVao_);
  glBindBuffer(GL_ARRAY_BUFFER, dynamicLineVbo_);
  glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(VertexPC)), vertices.data(), GL_DYNAMIC_DRAW);

  glLineWidth(width);
  glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertices.size()));
  glBindVertexArray(0);
}

void Renderer::DrawOverlay(const int /*width*/, const int height, const std::vector<OverlayLine>& lines) {
  glDisable(GL_DEPTH_TEST);

  float y = static_cast<float>(height) - 12.0F;
  constexpr float scale = 2.0F;
  for (const auto& line : lines) {
    DrawText2D(10.0F, y, scale, line.text, line.color);
    y -= 8.0F * scale;
  }

  glEnable(GL_DEPTH_TEST);
}

}  // namespace repulsor3d
