#include "repulsor3d/Renderer.hpp"

#include <GL/glew.h>

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>

#include <glm/common.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/trigonometric.hpp>

#include "repulsor3d/render/RenderCommandBuffer.hpp"
#include "repulsor3d/render/RenderGraph.hpp"
#include "repulsor3d/render/SceneModelBuilderFactory.hpp"

namespace repulsor3d {

Renderer::Renderer(const ViewerConfig& cfg, std::unique_ptr<ISceneModelBuilder> sceneBuilder)
    : cfg_(cfg),
      sceneBuilder_(std::move(sceneBuilder)),
      backend_(std::make_unique<OpenGLRenderBackend>()),
      assetResolver_(std::make_unique<DefaultSceneAssetResolver>()),
      geometryProvider_(std::make_unique<DefaultGeometryProvider>(*assetResolver_)) {
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

  renderFeatures_ = CreateDefaultRenderFeatures();
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

void Renderer::SetRenderFeatures(std::vector<std::unique_ptr<IRenderFeature>> renderFeatures) {
  if (!renderFeatures.empty()) {
    renderFeatures_ = std::move(renderFeatures);
  }
}

IRenderBackend& Renderer::GetRenderBackend() {
  return *backend_;
}

IGeometryProvider& Renderer::GetGeometryProvider() {
  return *geometryProvider_;
}

ISceneAssetResolver& Renderer::GetAssetResolver() {
  return *assetResolver_;
}

const DiagnosticsSnapshot& Renderer::LatestDiagnostics() const {
  return diagnostics_.Latest();
}

bool Renderer::Initialize() {
  backend_->ConfigureDefaultState();

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

  for (auto& feature : renderFeatures_) {
    if (feature == nullptr) {
      continue;
    }
    if (!feature->Initialize(*this)) {
      return false;
    }
  }

  Resize(width_, height_);
  return true;
}

void Renderer::Resize(const int w, const int h) {
  width_ = std::max(1, w);
  height_ = std::max(1, h);
  backend_->ResizeViewport(width_, height_);
}

void Renderer::Draw(GLFWwindow* /*window*/, const OrbitCamera& camera, const SnapshotBundle& bundle) {
  backend_->ClearFrame(glm::vec4{0.06F, 0.06F, 0.07F, 1.0F});

  SceneToggleState toggles;
  toggles.showCameraDebug = showCameraDebug;
  toggles.showTruthFuel = showTruthFuel;
  toggles.showAgeFilteredFuel = showAgeFilteredFuel;
  toggles.showFieldImage = showFieldImage;

  RenderSceneFrame sceneFrame = sceneBuilder_->BuildFrame(bundle, toggles);
  RenderCommandBuffer commandBuffer = BuildRenderCommandBuffer(sceneFrame);

  const float aspect = static_cast<float>(width_) / static_cast<float>(height_);
  const glm::mat4 vp = camera.ProjectionMatrix(aspect) * camera.ViewMatrix();
  const DiagnosticsSnapshot* diag = cfg_.showDiagnostics ? &diagnostics_.Latest() : nullptr;
  const RenderFeatureContext context{
      .viewProjection = vp,
      .viewportWidth = width_,
      .viewportHeight = height_,
      .frame = sceneFrame,
      .commandBuffer = commandBuffer,
      .diagnostics = diag,
  };
  const RendererDrawApi drawApi(*this);

  diagnostics_.BeginFrame();
  const auto frameStart = std::chrono::steady_clock::now();

  RenderGraph graph;
  for (const auto& feature : renderFeatures_) {
    if (feature == nullptr) {
      continue;
    }

    IRenderFeature* rawFeature = feature.get();
    graph.AddPass({
        .name = rawFeature->Name(),
        .dependencies = rawFeature->Dependencies(),
        .execute =
            [this, rawFeature, &context, &drawApi]() {
              const auto start = std::chrono::steady_clock::now();
              rawFeature->Render(context, drawApi);
              const auto end = std::chrono::steady_clock::now();
              const double ms =
                  std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start).count();
              diagnostics_.RecordFeatureTime(rawFeature->Name(), ms);
            },
    });
  }

  graph.Execute();
  const auto frameEnd = std::chrono::steady_clock::now();
  const double frameMs =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(frameEnd - frameStart).count();
  diagnostics_.EndFrame(frameMs);
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
  if (fieldTexture_ == 0) {
    return;
  }

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

void Renderer::DrawLinePrimitives(const glm::mat4& vp, const std::vector<LinePrimitive>& lines) {
  if (lines.empty()) {
    return;
  }

  std::map<float, std::vector<VertexPC>> lineBatches;
  for (const auto& line : lines) {
    auto& batch = lineBatches[line.width];
    batch.push_back({line.a, line.color});
    batch.push_back({line.b, line.color});
  }

  for (const auto& [width, verts] : lineBatches) {
    DrawLineList(vp, verts, width);
  }
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
  backend_->SetDepthTestEnabled(false);

  float y = static_cast<float>(height) - 12.0F;
  constexpr float scale = 2.0F;
  for (const auto& line : lines) {
    DrawText2D(10.0F, y, scale, line.text, line.color);
    y -= 8.0F * scale;
  }

  backend_->SetDepthTestEnabled(true);
}

}  // namespace repulsor3d
