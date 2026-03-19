#include "repulsor3d/Renderer.hpp"

#include <GL/glew.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>

#include <glm/common.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/trigonometric.hpp>

#include "repulsor3d/render/RenderCommandBuffer.hpp"
#include "repulsor3d/render/RenderGraph.hpp"
#include "repulsor3d/render/ecs/Systems.hpp"

namespace repulsor3d {
namespace {

bool HasMeshInstanceCommands(const RenderCommandBuffer& commandBuffer) {
  for (const auto& command : commandBuffer) {
    bool hasMesh = false;
    std::visit(
        [&](const auto& typed) {
          using CommandType = std::decay_t<decltype(typed)>;
          hasMesh = std::is_same_v<CommandType, DrawMeshInstanceCommand>;
        },
        command);
    if (hasMesh) {
      return true;
    }
  }
  return false;
}

}  // namespace

Renderer::Renderer(const ViewerConfig& cfg, std::unique_ptr<IRenderWorldAdapter> worldAdapter)
    : cfg_(cfg),
      worldAdapter_(std::move(worldAdapter)),
      backend_(std::make_unique<OpenGLRenderBackend>()),
      assetResolver_(std::make_unique<DefaultSceneAssetResolver>()),
      geometryProvider_(std::make_unique<DefaultGeometryProvider>(*assetResolver_)) {
  resourceManager_.SetBudget(ResourceClass::Buffer, ResourceBudget{.maxCount = 4096, .maxBytes = 0});
  resourceManager_.SetBudget(ResourceClass::Texture, ResourceBudget{.maxCount = 512, .maxBytes = 0});
  resourceManager_.SetBudget(ResourceClass::VertexArray, ResourceBudget{.maxCount = 2048, .maxBytes = 0});

  showCameraDebug = cfg.showCameraDebug;
  showTruthFuel = cfg.showTruthFuel;
  showAgeFilteredFuel = cfg.showAgeFilteredFuel;
  showFieldImage = cfg.showFieldImage;
  showDebugPanel = cfg.showDebugPanel;

  width_ = std::max(1, cfg.windowW);
  height_ = std::max(1, cfg.windowH);

  fieldLength_ = cfg.fieldLengthM;
  fieldWidth_ = cfg.fieldWidthM;
  fieldZ_ = cfg.fieldZM;

  if (worldAdapter_ == nullptr) {
    worldAdapter_ = CreateDefaultRenderWorldAdapter(cfg_);
  }

  renderFeatures_ = CreateDefaultRenderFeatures(cfg_);
  renderPipelinePath_ = cfg_.renderPipelinePath;
  renderFeaturePluginPath_ = cfg_.renderFeaturePluginPath;
}

Renderer::~Renderer() {
  DestroyShader(solidShader_);
  DestroyShader(lineShader_);
  DestroyShader(texturedShader_);

  DestroyMesh(cubeMesh_);
  DestroyMesh(sphereMesh_);
  DestroyMesh(quadMesh_);

  for (auto& [_, timer] : gpuPassTimers_) {
    for (unsigned int& queryId : timer.queryIds) {
      if (queryId != 0) {
        backend_->DestroyGpuTimerQuery(queryId);
        queryId = 0;
      }
    }
  }

  if (dynamicLineVbo_.Get() != 0) {
    resourceManager_.Release(ResourceClass::Buffer, "renderer.dynamic_lines.vbo");
  }
  if (textVbo_.Get() != 0) {
    resourceManager_.Release(ResourceClass::Buffer, "renderer.text.vbo");
  }
  if (dynamicLineVao_.Get() != 0) {
    resourceManager_.Release(ResourceClass::VertexArray, "renderer.dynamic_lines.vao");
  }
  if (textVao_.Get() != 0) {
    resourceManager_.Release(ResourceClass::VertexArray, "renderer.text.vao");
  }
  if (fieldTexture_.Get() != 0) {
    resourceManager_.Release(ResourceClass::Texture, "renderer.field_texture");
  }
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
  if (renderFeatures.empty()) {
    return;
  }
  if (renderFeaturesInitialized_) {
    for (auto& feature : renderFeatures) {
      if (feature != nullptr && !feature->Initialize(*this)) {
        return;
      }
    }
  }
  renderFeatures_ = std::move(renderFeatures);
  renderFeaturesInitialized_ = true;
}

void Renderer::ApplyRuntimeConfig(const ViewerConfig& cfg) {
  const bool pipelineChanged = cfg_.renderPipelinePath != cfg.renderPipelinePath;
  const bool pluginChanged = cfg_.renderFeaturePluginPath != cfg.renderFeaturePluginPath;

  cfg_ = cfg;
  fieldLength_ = cfg_.fieldLengthM;
  fieldWidth_ = cfg_.fieldWidthM;
  fieldZ_ = cfg_.fieldZM;
  showCameraDebug = cfg_.showCameraDebug;
  showTruthFuel = cfg_.showTruthFuel;
  showAgeFilteredFuel = cfg_.showAgeFilteredFuel;
  showFieldImage = cfg_.showFieldImage;
  showDebugPanel = cfg_.showDebugPanel;

  if ((pipelineChanged || pluginChanged) && renderFeaturesInitialized_) {
    RebuildRenderFeatures("runtime_config");
  }
}

void Renderer::QueueDiagnosticMessage(std::string message) {
  if (message.empty()) {
    return;
  }
  queuedDiagnosticMessages_.push_back(std::move(message));
  constexpr std::size_t kMaxQueuedMessages = 16;
  if (queuedDiagnosticMessages_.size() > kMaxQueuedMessages) {
    queuedDiagnosticMessages_.erase(
        queuedDiagnosticMessages_.begin(),
        queuedDiagnosticMessages_.begin() + static_cast<std::ptrdiff_t>(queuedDiagnosticMessages_.size() - kMaxQueuedMessages));
  }
}

void Renderer::QueueDiagnosticCounter(std::string name, const double value) {
  if (name.empty()) {
    return;
  }
  queuedDiagnosticCounters_.push_back({std::move(name), value});
  constexpr std::size_t kMaxQueuedCounters = 64;
  if (queuedDiagnosticCounters_.size() > kMaxQueuedCounters) {
    queuedDiagnosticCounters_.erase(
        queuedDiagnosticCounters_.begin(),
        queuedDiagnosticCounters_.begin() + static_cast<std::ptrdiff_t>(queuedDiagnosticCounters_.size() - kMaxQueuedCounters));
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
  resourceManager_.Register(ResourceClass::VertexArray, "renderer.dynamic_lines.vao");
  resourceManager_.Register(ResourceClass::Buffer, "renderer.dynamic_lines.vbo", 1024 * sizeof(VertexPC));
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
  resourceManager_.Register(ResourceClass::VertexArray, "renderer.text.vao");
  resourceManager_.Register(ResourceClass::Buffer, "renderer.text.vbo", 4096 * sizeof(glm::vec3));
  textVao_.Set(textVao);
  textVbo_.Set(textVbo);

  backend_->BindVertexArray(textVao_.Get());
  backend_->BindArrayBuffer(textVbo_.Get());
  backend_->UploadArrayBufferData(4096 * sizeof(glm::vec3), nullptr, true);
  backend_->EnableVertexAttrib(0);
  backend_->DefineVertexAttribFloat(0, 3, sizeof(glm::vec3), 0);
  backend_->BindVertexArray(0);
  if (!RebuildRenderFeatures("startup")) {
    return false;
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

  const float aspect = static_cast<float>(width_) / static_cast<float>(height_);
  const glm::mat4 view = camera.ViewMatrix();
  const glm::mat4 projection = camera.ProjectionMatrix(aspect);
  const glm::mat4 vp = projection * view;
  const EntityCullingStats cullingStats = ApplyRenderEntityHierarchyAndCulling(sceneFrame, vp);
  RenderCommandBuffer commandBuffer = BuildRenderCommandBuffer(sceneFrame);
  if (!HasMeshInstanceCommands(commandBuffer)) {
    diagnostics_.MarkSceneReady();
  }

  const DiagnosticsSnapshot* diag = cfg_.showDiagnostics ? &diagnostics_.Latest() : nullptr;
  const RenderFeatureContext context{
      .viewProjection = vp,
      .cameraWorldPosition = camera.Eye(),
      .viewportWidth = width_,
      .viewportHeight = height_,
      .frame = sceneFrame,
      .commandBuffer = commandBuffer,
      .diagnostics = diag,
      .diagnosticsWriter = &diagnostics_,
      .showDebugPanel = showDebugPanel,
      .showDebugCounters = showDebugCounters,
      .showDebugCpu = showDebugCpu,
      .showDebugGpu = showDebugGpu,
      .showDebugAssets = showDebugAssets,
  };
  const RendererDrawApi drawApi(*this);

  diagnostics_.BeginFrame();
  diagnostics_.RecordCounter("entities.total", cullingStats.totalEntities);
  diagnostics_.RecordCounter("entities.candidates", cullingStats.candidates);
  diagnostics_.RecordCounter("entities.visible", cullingStats.visible);
  diagnostics_.RecordCounter("entities.culled", cullingStats.culled);
  diagnostics_.RecordCounter("commands.total", static_cast<double>(commandBuffer.size()));
  const auto& caps = backend_->Capabilities();
  diagnostics_.RecordCounter("backend.max_texture_size", static_cast<double>(caps.maxTextureSize));
  diagnostics_.RecordCounter("backend.max_vertex_attribs", static_cast<double>(caps.maxVertexAttribs));
  diagnostics_.RecordCounter("backend.supports_gpu_timers", caps.supportsGpuTimers ? 1.0 : 0.0);
  diagnostics_.RecordCounter("resources.buffers", static_cast<double>(resourceManager_.Stats(ResourceClass::Buffer).count));
  diagnostics_.RecordCounter("resources.textures", static_cast<double>(resourceManager_.Stats(ResourceClass::Texture).count));
  diagnostics_.RecordCounter("resources.vertex_arrays", static_cast<double>(resourceManager_.Stats(ResourceClass::VertexArray).count));
  diagnostics_.RecordCounter(
      "resources.over_budget",
      (resourceManager_.IsOverBudget(ResourceClass::Buffer) ||
       resourceManager_.IsOverBudget(ResourceClass::Texture) ||
       resourceManager_.IsOverBudget(ResourceClass::VertexArray))
          ? 1.0
          : 0.0);
  diagnostics_.RecordCounter(
      "resources.over_budget_count",
      static_cast<double>(
          resourceManager_.OverBudgetCount(ResourceClass::Buffer) +
          resourceManager_.OverBudgetCount(ResourceClass::Texture) +
          resourceManager_.OverBudgetCount(ResourceClass::VertexArray)));
  diagnostics_.RecordCounter(
      "resources.over_budget_bytes",
      static_cast<double>(
          resourceManager_.OverBudgetBytes(ResourceClass::Buffer) +
          resourceManager_.OverBudgetBytes(ResourceClass::Texture) +
          resourceManager_.OverBudgetBytes(ResourceClass::VertexArray)));
  diagnostics_.RecordCounter("render_pipeline.reload_count", static_cast<double>(renderFeatureReloadCount_));
  diagnostics_.RecordCounter("render_pipeline.reload_failed_count", static_cast<double>(renderFeatureReloadFailedCount_));
  diagnostics_.RecordCounter("render_pipeline.reload_has_error", renderFeatureReloadLastError_.empty() ? 0.0 : 1.0);
  diagnostics_.RecordCounter("render_pipeline.last_reload_ok", renderFeatureReloadLastError_.empty() ? 1.0 : 0.0);
  for (const auto& [name, value] : queuedDiagnosticCounters_) {
    diagnostics_.RecordCounter(name, value);
  }
  queuedDiagnosticCounters_.clear();
  for (const auto& message : queuedDiagnosticMessages_) {
    diagnostics_.RecordMessage(message);
  }
  queuedDiagnosticMessages_.clear();
  const auto frameStart = std::chrono::steady_clock::now();

  MaybeHotReloadRenderFeatures();

  RenderGraph graph;
  for (const auto& feature : renderFeatures_) {
    if (feature == nullptr) {
      continue;
    }

    IRenderFeature* rawFeature = feature.get();
    graph.AddPass({
        .name = rawFeature->Name(),
        .dependencies = rawFeature->Dependencies(),
        .consumesResources = rawFeature->Dependencies(),
        .producesResources = {rawFeature->Name()},
        .execute =
            [this, rawFeature, &context, &drawApi](RenderGraphContext& graphContext) {
              auto& gpuTimer = gpuPassTimers_[rawFeature->Name()];
              if (gpuTimer.queryIds[0] == 0) {
                gpuTimer.queryIds[0] = backend_->CreateGpuTimerQuery();
              }
              if (gpuTimer.queryIds[1] == 0) {
                gpuTimer.queryIds[1] = backend_->CreateGpuTimerQuery();
              }

              const unsigned int readQuery = gpuTimer.queryIds[gpuTimer.readIndex];
              if (readQuery != 0) {
                double gpuMs = 0.0;
                if (backend_->TryReadGpuTimerMilliseconds(readQuery, gpuMs)) {
                  diagnostics_.RecordGpuTime(rawFeature->Name(), gpuMs);
                }
              }

              const unsigned int writeQuery = gpuTimer.queryIds[gpuTimer.writeIndex];
              const bool gpuQueryActive = writeQuery != 0;
              if (gpuQueryActive) {
                backend_->BeginGpuTimerQuery(writeQuery);
              }

              const auto start = std::chrono::steady_clock::now();
              rawFeature->Render(context, drawApi);
              const auto end = std::chrono::steady_clock::now();
              if (gpuQueryActive) {
                backend_->EndGpuTimerQuery();
              }

              const double ms =
                  std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start).count();
              diagnostics_.RecordPassTime(rawFeature->Name(), ms);

              std::swap(gpuTimer.readIndex, gpuTimer.writeIndex);
              graphContext.MarkResourceAvailable(rawFeature->Name());
            },
    });
  }

  graph.Execute();
  DrainAssetTelemetry();

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

bool Renderer::QueryFileWriteTime(const std::string& path, std::filesystem::file_time_type& outWriteTime) {
  if (path.empty()) {
    return false;
  }
  std::error_code ec;
  if (!std::filesystem::exists(path, ec) || ec) {
    return false;
  }
  outWriteTime = std::filesystem::last_write_time(path, ec);
  return !ec;
}

void Renderer::CaptureRenderFeatureInputStamps() {
  renderPipelinePath_ = cfg_.renderPipelinePath;
  renderFeaturePluginPath_ = cfg_.renderFeaturePluginPath;
  renderPipelineWriteTimeKnown_ = QueryFileWriteTime(renderPipelinePath_, renderPipelineWriteTime_);
  renderFeaturePluginWriteTimeKnown_ = QueryFileWriteTime(renderFeaturePluginPath_, renderFeaturePluginWriteTime_);
}

bool Renderer::RebuildRenderFeatures(std::string reason) {
  auto features = CreateDefaultRenderFeatures(cfg_);
  std::unique_ptr<IRenderFeaturePlugin> plugin;
  if (!cfg_.renderFeaturePluginPath.empty()) {
    plugin = CreateRenderFeaturePluginFromPath(cfg_.renderFeaturePluginPath);
    if (plugin == nullptr) {
      renderFeatureReloadLastError_ = "failed to load render plugin '" + cfg_.renderFeaturePluginPath + "'";
      ++renderFeatureReloadFailedCount_;
      std::cerr << "[Renderer] " << renderFeatureReloadLastError_ << "\n";
      QueueDiagnosticMessage("render pipeline reload failed: " + renderFeatureReloadLastError_);
      CaptureRenderFeatureInputStamps();
      return false;
    }
    auto pluginFeatures = plugin->CreateFeatures(cfg_);
    for (auto& feature : pluginFeatures) {
      if (feature != nullptr) {
        features.push_back(std::move(feature));
      }
    }
  }

  if (features.empty()) {
    renderFeatureReloadLastError_ = "render feature list is empty";
    ++renderFeatureReloadFailedCount_;
    std::cerr << "[Renderer] " << renderFeatureReloadLastError_ << "\n";
    QueueDiagnosticMessage("render pipeline reload failed: " + renderFeatureReloadLastError_);
    CaptureRenderFeatureInputStamps();
    return false;
  }

  for (auto& feature : features) {
    if (feature == nullptr) {
      continue;
    }
    if (!feature->Initialize(*this)) {
      renderFeatureReloadLastError_ = "feature init failed: " + feature->Name();
      ++renderFeatureReloadFailedCount_;
      std::cerr << "[Renderer] " << renderFeatureReloadLastError_ << "\n";
      QueueDiagnosticMessage("render pipeline reload failed: " + renderFeatureReloadLastError_);
      CaptureRenderFeatureInputStamps();
      return false;
    }
  }

  renderFeatures_ = std::move(features);
  renderFeaturePlugin_ = std::move(plugin);
  renderFeaturesInitialized_ = true;
  ++renderFeatureReloadCount_;
  renderFeatureReloadLastError_.clear();
  CaptureRenderFeatureInputStamps();
  std::cerr << "[Renderer] render features rebuilt (" << reason << ")\n";
  QueueDiagnosticMessage("render features rebuilt (" + reason + ")");
  return true;
}

void Renderer::MaybeHotReloadRenderFeatures() {
  if (!renderFeaturesInitialized_) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (nextRenderFeatureReloadCheck_.time_since_epoch().count() != 0 && now < nextRenderFeatureReloadCheck_) {
    return;
  }
  nextRenderFeatureReloadCheck_ = now + std::chrono::milliseconds(500);

  const bool pipelinePathChanged = renderPipelinePath_ != cfg_.renderPipelinePath;
  const bool pluginPathChanged = renderFeaturePluginPath_ != cfg_.renderFeaturePluginPath;
  bool shouldReload = pipelinePathChanged || pluginPathChanged;

  std::filesystem::file_time_type pipelineStamp{};
  const bool pipelineStampKnown = QueryFileWriteTime(cfg_.renderPipelinePath, pipelineStamp);
  if (!pipelinePathChanged) {
    if (pipelineStampKnown != renderPipelineWriteTimeKnown_) {
      shouldReload = true;
    } else if (pipelineStampKnown && pipelineStamp != renderPipelineWriteTime_) {
      shouldReload = true;
    }
  }

  std::filesystem::file_time_type pluginStamp{};
  const bool pluginStampKnown = QueryFileWriteTime(cfg_.renderFeaturePluginPath, pluginStamp);
  if (!pluginPathChanged) {
    if (pluginStampKnown != renderFeaturePluginWriteTimeKnown_) {
      shouldReload = true;
    } else if (pluginStampKnown && pluginStamp != renderFeaturePluginWriteTime_) {
      shouldReload = true;
    }
  }

  if (!shouldReload) {
    return;
  }
  RebuildRenderFeatures("file_watch");
}

void Renderer::DrainAssetTelemetry() {
  if (geometryProvider_ == nullptr) {
    return;
  }

  for (const auto& event : geometryProvider_->ConsumeCadAssetTelemetry()) {
    std::filesystem::path p(event.assetPath);
    std::string label = event.eventName;
    if (!p.empty()) {
      label += " [" + p.filename().string() + "]";
    }
    diagnostics_.RecordAssetTime(label, event.milliseconds);
  }
}

void Renderer::DrawGrid(const glm::mat4& vp) {
  std::vector<VertexPC> lines;
  lines.reserve(512);

  const glm::vec4 col{0.15F, 0.15F, 0.18F, 1.0F};
  const float step = 1.0F;
  const float halfLength = fieldLength_ * 0.5F;
  const float halfWidth = fieldWidth_ * 0.5F;

  for (float x = -halfLength; x <= halfLength + 1e-4F; x += step) {
    lines.push_back({glm::vec3{x, -halfWidth, fieldZ_}, col});
    lines.push_back({glm::vec3{x, halfWidth, fieldZ_}, col});
  }
  for (float y = -halfWidth; y <= halfWidth + 1e-4F; y += step) {
    lines.push_back({glm::vec3{-halfLength, y, fieldZ_}, col});
    lines.push_back({glm::vec3{halfLength, y, fieldZ_}, col});
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
