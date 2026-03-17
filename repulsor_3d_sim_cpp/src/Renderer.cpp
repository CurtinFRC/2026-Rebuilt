#include "repulsor3d/Renderer.hpp"

#include <GL/glew.h>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>

#include <glm/common.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/trigonometric.hpp>

#include "repulsor3d/render/RenderCommandBuffer.hpp"
#include "repulsor3d/render/RenderGraph.hpp"

namespace repulsor3d {

Renderer::Renderer(const ViewerConfig& cfg, std::unique_ptr<IRenderWorldAdapter> worldAdapter)
    : cfg_(cfg),
      worldAdapter_(std::move(worldAdapter)),
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

  if (worldAdapter_ == nullptr) {
    worldAdapter_ = CreateDefaultRenderWorldAdapter(cfg_);
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
}

void Renderer::SetRenderWorldAdapter(std::unique_ptr<IRenderWorldAdapter> worldAdapter) {
  if (worldAdapter != nullptr) {
    worldAdapter_ = std::move(worldAdapter);
  }
}

void Renderer::SetSceneModelBuilder(std::unique_ptr<ISceneModelBuilder> sceneBuilder) {
  if (sceneBuilder != nullptr) {
    worldAdapter_ = CreateRenderWorldAdapterFromSceneBuilder(std::move(sceneBuilder));
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

  const unsigned int dynamicLineVao = backend_->CreateVertexArray();
  const unsigned int dynamicLineVbo = backend_->CreateBuffer();
  dynamicLineVao_.Set(dynamicLineVao);
  dynamicLineVbo_.Set(dynamicLineVbo);

  backend_->BindVertexArray(dynamicLineVao_.Get());
  backend_->BindArrayBuffer(dynamicLineVbo_.Get());
  backend_->UploadArrayBufferData(1024 * sizeof(VertexPC), nullptr, true);
  backend_->EnableVertexAttrib(0);
  backend_->DefineVertexAttribFloat(0, 3, sizeof(VertexPC), offsetof(VertexPC, pos));
  backend_->EnableVertexAttrib(1);
  backend_->DefineVertexAttribFloat(1, 4, sizeof(VertexPC), offsetof(VertexPC, color));
  backend_->BindVertexArray(0);

  const unsigned int textVao = backend_->CreateVertexArray();
  const unsigned int textVbo = backend_->CreateBuffer();
  textVao_.Set(textVao);
  textVbo_.Set(textVbo);

  backend_->BindVertexArray(textVao_.Get());
  backend_->BindArrayBuffer(textVbo_.Get());
  backend_->UploadArrayBufferData(4096 * sizeof(glm::vec3), nullptr, true);
  backend_->EnableVertexAttrib(0);
  backend_->DefineVertexAttribFloat(0, 3, sizeof(glm::vec3), 0);
  backend_->BindVertexArray(0);

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

void Renderer::Draw(GLFWwindow* window, const OrbitCamera& camera, const SnapshotBundle& bundle) {
  SnapshotBundleSimWorld world(bundle);
  Draw(window, camera, world);
}

void Renderer::Draw(GLFWwindow* /*window*/, const OrbitCamera& camera, const ISimWorld& world) {
  backend_->ClearFrame(glm::vec4{0.06F, 0.06F, 0.07F, 1.0F});

  SceneToggleState toggles;
  toggles.showCameraDebug = showCameraDebug;
  toggles.showTruthFuel = showTruthFuel;
  toggles.showAgeFilteredFuel = showAgeFilteredFuel;
  toggles.showFieldImage = showFieldImage;

  RenderSceneFrame sceneFrame;
  if (worldAdapter_ != nullptr) {
    sceneFrame = worldAdapter_->BuildFrame(world, toggles);
  }
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
  if (frameMs > 0.0) {
    constexpr double alpha = 0.10;
    if (!smoothedFrameMsInitialized_) {
      smoothedFrameMs_ = frameMs;
      smoothedFrameMsInitialized_ = true;
    } else {
      smoothedFrameMs_ = (1.0 - alpha) * smoothedFrameMs_ + alpha * frameMs;
    }
  }
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
  if (fieldTexture_.Get() == 0) {
    return;
  }

  backend_->UseProgram(texturedShader_.program.Get());

  glm::mat4 model(1.0F);
  model = glm::translate(model, glm::vec3{0.0F, 0.0F, fieldZ_ + 0.001F});
  model = glm::scale(model, glm::vec3{fieldLength_, fieldWidth_, 1.0F});

  const glm::mat4 mvp = vp * model;
  const unsigned int texProgram = texturedShader_.program.Get();
  const int mvpLoc = backend_->GetUniformLocation(texProgram, "uMvp");
  const int alphaLoc = backend_->GetUniformLocation(texProgram, "uAlpha");
  const int flipXLoc = backend_->GetUniformLocation(texProgram, "uFlipX");
  const int flipYLoc = backend_->GetUniformLocation(texProgram, "uFlipY");
  const int texLoc = backend_->GetUniformLocation(texProgram, "uTex");
  backend_->SetUniformMat4(mvpLoc, &mvp[0][0]);
  backend_->SetUniform1f(alphaLoc, glm::clamp(cfg_.fieldImageAlpha, 0.0F, 1.0F));
  backend_->SetUniform1i(flipXLoc, cfg_.fieldImageFlipX ? 1 : 0);
  backend_->SetUniform1i(flipYLoc, cfg_.fieldImageFlipY ? 1 : 0);

  backend_->SetActiveTextureUnit(0);
  backend_->BindTexture2D(fieldTexture_.Get());
  backend_->SetUniform1i(texLoc, 0);

  backend_->BindVertexArray(quadMesh_.vao.Get());
  backend_->DrawIndexedTriangles(quadMesh_.indexCount);
  backend_->BindVertexArray(0);
  backend_->BindTexture2D(0);
}

void Renderer::DrawBox(const glm::mat4& vp, const BoxPrimitive& primitive) {
  glm::mat4 model(1.0F);
  model = glm::translate(model, primitive.center);
  model = glm::rotate(model, glm::radians(primitive.yawDeg), glm::vec3{0.0F, 0.0F, 1.0F});
  model = glm::scale(model, primitive.size);

  const glm::mat4 mvp = vp * model;

  const unsigned int solidProgram = solidShader_.program.Get();
  backend_->UseProgram(solidProgram);
  const int mvpLoc = backend_->GetUniformLocation(solidProgram, "uMvp");
  const int colorLoc = backend_->GetUniformLocation(solidProgram, "uColor");
  backend_->SetUniformMat4(mvpLoc, &mvp[0][0]);
  backend_->SetUniformVec4(colorLoc, primitive.color.r, primitive.color.g, primitive.color.b, primitive.color.a);

  backend_->BindVertexArray(cubeMesh_.vao.Get());
  backend_->DrawIndexedTriangles(cubeMesh_.indexCount);
  backend_->BindVertexArray(0);
}

void Renderer::DrawSphere(const glm::mat4& vp, const SpherePrimitive& primitive) {
  glm::mat4 model(1.0F);
  model = glm::translate(model, primitive.center);
  model = glm::scale(model, glm::vec3{primitive.radius, primitive.radius, primitive.radius});

  const glm::mat4 mvp = vp * model;

  const unsigned int solidProgram = solidShader_.program.Get();
  backend_->UseProgram(solidProgram);
  const int mvpLoc = backend_->GetUniformLocation(solidProgram, "uMvp");
  const int colorLoc = backend_->GetUniformLocation(solidProgram, "uColor");
  backend_->SetUniformMat4(mvpLoc, &mvp[0][0]);
  backend_->SetUniformVec4(colorLoc, primitive.color.r, primitive.color.g, primitive.color.b, primitive.color.a);

  backend_->BindVertexArray(sphereMesh_.vao.Get());
  backend_->DrawIndexedTriangles(sphereMesh_.indexCount);
  backend_->BindVertexArray(0);
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

  const unsigned int lineProgram = lineShader_.program.Get();
  backend_->UseProgram(lineProgram);
  const int mvpLoc = backend_->GetUniformLocation(lineProgram, "uMvp");
  backend_->SetUniformMat4(mvpLoc, &vp[0][0]);

  backend_->BindVertexArray(dynamicLineVao_.Get());
  backend_->BindArrayBuffer(dynamicLineVbo_.Get());
  backend_->UploadArrayBufferData(vertices.size() * sizeof(VertexPC), vertices.data(), true);

  backend_->DrawLines(static_cast<int>(vertices.size()), width);
  backend_->BindVertexArray(0);
}

void Renderer::DrawOverlay(const int width, const int height, const std::vector<OverlayLine>& lines) {
  backend_->SetDepthTestEnabled(false);

  constexpr float scale = 2.0F;
  constexpr float lineStep = 8.0F * scale;
  constexpr float glyphHeight = 7.0F * scale;
  std::vector<OverlayLine> widgets = lines;

  if (smoothedFrameMsInitialized_ && smoothedFrameMs_ > 1e-6) {
    std::ostringstream fpsText;
    fpsText << std::fixed << std::setprecision(1) << "FPS: " << (1000.0 / smoothedFrameMs_);
    widgets.push_back(
        {.text = fpsText.str(),
         .color = glm::vec4{0.92F, 0.92F, 0.92F, 0.90F},
         .anchor = OverlayAnchor::TopRight,
         .marginX = 10.0F,
         .marginY = 12.0F});
  }

  bool hasTopLeft = false;
  bool hasTopRight = false;
  bool hasBottomLeft = false;
  bool hasBottomRight = false;
  float topLeftY = 0.0F;
  float topRightY = 0.0F;
  float bottomLeftY = 0.0F;
  float bottomRightY = 0.0F;

  for (const auto& line : widgets) {
    const float marginX = std::max(0.0F, line.marginX);
    const float marginY = std::max(0.0F, line.marginY);
    const float textWidth = TextWidthPixels(line.text, scale);

    float x = marginX;
    float y = static_cast<float>(height) - marginY;

    switch (line.anchor) {
      case OverlayAnchor::TopLeft:
        if (!hasTopLeft) {
          topLeftY = static_cast<float>(height) - marginY;
          hasTopLeft = true;
        }
        x = marginX;
        y = topLeftY;
        topLeftY -= lineStep;
        break;
      case OverlayAnchor::TopRight:
        if (!hasTopRight) {
          topRightY = static_cast<float>(height) - marginY;
          hasTopRight = true;
        }
        x = static_cast<float>(width) - marginX - textWidth;
        y = topRightY;
        topRightY -= lineStep;
        break;
      case OverlayAnchor::BottomLeft:
        if (!hasBottomLeft) {
          bottomLeftY = marginY + glyphHeight;
          hasBottomLeft = true;
        }
        x = marginX;
        y = bottomLeftY;
        bottomLeftY += lineStep;
        break;
      case OverlayAnchor::BottomRight:
        if (!hasBottomRight) {
          bottomRightY = marginY + glyphHeight;
          hasBottomRight = true;
        }
        x = static_cast<float>(width) - marginX - textWidth;
        y = bottomRightY;
        bottomRightY += lineStep;
        break;
    }

    DrawText2D(std::max(0.0F, x), y, scale, line.text, line.color);
  }

  backend_->SetDepthTestEnabled(true);
}

}  // namespace repulsor3d
