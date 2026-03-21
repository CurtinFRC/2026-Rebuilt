#include "repulsor3d/Renderer.hpp"

#include <GL/glew.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <cstdlib>
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

glm::vec4 LerpColor(const glm::vec4& a, const glm::vec4& b, const float tRaw) {
  const float t = glm::clamp(tRaw, 0.0F, 1.0F);
  return a * (1.0F - t) + b * t;
}

float Hash01(std::uint32_t x) {
  x ^= x >> 16U;
  x *= 0x7feb352dU;
  x ^= x >> 15U;
  x *= 0x846ca68bU;
  x ^= x >> 16U;
  return static_cast<float>(x & 0x00FFFFFFU) / static_cast<float>(0x01000000U);
}

bool ParseEnvBool(const char* name, const bool fallback) {
  if (name == nullptr || *name == '\0') {
    return fallback;
  }
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  std::string text(value);
  std::transform(text.begin(), text.end(), text.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (text == "1" || text == "true" || text == "yes" || text == "on") {
    return true;
  }
  if (text == "0" || text == "false" || text == "no" || text == "off") {
    return false;
  }
  return fallback;
}

int ParseEnvInt(const char* name, const int fallback) {
  if (name == nullptr || *name == '\0') {
    return fallback;
  }
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  try {
    return std::stoi(value);
  } catch (...) {
    return fallback;
  }
}

float ParseEnvFloat(const char* name, const float fallback) {
  if (name == nullptr || *name == '\0') {
    return fallback;
  }
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  try {
    return std::stof(value);
  } catch (...) {
    return fallback;
  }
}

double FindCounterValue(const DiagnosticsSnapshot& snapshot, const char* name, const double fallback = 0.0) {
  if (name == nullptr) {
    return fallback;
  }
  for (const auto& counter : snapshot.counters) {
    if (counter.name == name) {
      return counter.value;
    }
  }
  return fallback;
}

const char* CadLoadStageLabelFromCode(const int stageCode) {
  switch (stageCode) {
    case 1:
      return "queued";
    case 2:
      return "cache_lookup";
    case 12:
      return "cache_lookup_slow";
    case 3:
      return "cache_hit";
    case 4:
      return "import";
    case 11:
      return "import_slow";
    case 5:
      return "prepare";
    case 13:
      return "prepare_slow";
    case 6:
      return "store_cache";
    case 14:
      return "store_cache_slow";
    case 7:
      return "gpu_upload";
    case 8:
      return "completed";
    case 9:
      return "cancelled";
    case 10:
      return "failed";
    default:
      return "idle";
  }
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
  DestroyBloomResources();
  DestroyShader(bloomCompositeShader_);
  DestroyShader(bloomBlurShader_);
  DestroyShader(bloomExtractShader_);
  DestroyShader(solidShader_);
  DestroyShader(lineShader_);
  DestroyShader(texturedShader_);
  DestroyShader(skyShader_);

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
  const bool sizeChanged = (width_ != std::max(1, w)) || (height_ != std::max(1, h));
  width_ = std::max(1, w);
  height_ = std::max(1, h);
  backend_->ResizeViewport(width_, height_);
  if (sizeChanged) {
    DestroyBloomResources();
  }
}

void Renderer::DestroyBloomResources() {
  if (sceneFramebuffer_ != 0) {
    glDeleteFramebuffers(1, &sceneFramebuffer_);
    sceneFramebuffer_ = 0;
  }
  if (bloomPingFramebuffer_ != 0) {
    glDeleteFramebuffers(1, &bloomPingFramebuffer_);
    bloomPingFramebuffer_ = 0;
  }
  if (bloomPongFramebuffer_ != 0) {
    glDeleteFramebuffers(1, &bloomPongFramebuffer_);
    bloomPongFramebuffer_ = 0;
  }
  sceneColorTexture_.Reset();
  sceneDepthTexture_.Reset();
  bloomPingTexture_.Reset();
  bloomPongTexture_.Reset();
  bloomSourceWidth_ = 0;
  bloomSourceHeight_ = 0;
  bloomWidth_ = 0;
  bloomHeight_ = 0;
  bloomResourcesReady_ = false;
}

bool Renderer::EnsureBloomResources(const int width, const int height) {
  const int downsample = std::clamp(ParseEnvInt("RENDER_BLOOM_DOWNSAMPLE", 2), 1, 4);
  const int bloomW = std::max(1, width / downsample);
  const int bloomH = std::max(1, height / downsample);
  if (bloomResourcesReady_ &&
      bloomSourceWidth_ == width &&
      bloomSourceHeight_ == height &&
      bloomWidth_ == bloomW &&
      bloomHeight_ == bloomH &&
      bloomDownsample_ == downsample) {
    return true;
  }

  DestroyBloomResources();
  bloomDownsample_ = downsample;

  auto createColorTexture = [&](const int texW, const int texH, GlTextureHandle& outTexture) -> bool {
    const unsigned int textureId = backend_->CreateTexture2D();
    if (textureId == 0) {
      return false;
    }
    outTexture.Set(textureId);
    glBindTexture(GL_TEXTURE_2D, outTexture.Get());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, texW, texH, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glBindTexture(GL_TEXTURE_2D, 0);
    return true;
  };

  if (!createColorTexture(width, height, sceneColorTexture_)) {
    DestroyBloomResources();
    return false;
  }
  if (!createColorTexture(bloomW, bloomH, bloomPingTexture_)) {
    DestroyBloomResources();
    return false;
  }
  if (!createColorTexture(bloomW, bloomH, bloomPongTexture_)) {
    DestroyBloomResources();
    return false;
  }

  glGenFramebuffers(1, &sceneFramebuffer_);
  glBindFramebuffer(GL_FRAMEBUFFER, sceneFramebuffer_);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, sceneColorTexture_.Get(), 0);
  const unsigned int depthTextureId = backend_->CreateTexture2D();
  if (depthTextureId == 0) {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    DestroyBloomResources();
    return false;
  }
  sceneDepthTexture_.Set(depthTextureId);
  glBindTexture(GL_TEXTURE_2D, sceneDepthTexture_.Get());
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, sceneDepthTexture_.Get(), 0);
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    DestroyBloomResources();
    return false;
  }

  glGenFramebuffers(1, &bloomPingFramebuffer_);
  glBindFramebuffer(GL_FRAMEBUFFER, bloomPingFramebuffer_);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, bloomPingTexture_.Get(), 0);
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    DestroyBloomResources();
    return false;
  }

  glGenFramebuffers(1, &bloomPongFramebuffer_);
  glBindFramebuffer(GL_FRAMEBUFFER, bloomPongFramebuffer_);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, bloomPongTexture_.Get(), 0);
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    DestroyBloomResources();
    return false;
  }

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glBindTexture(GL_TEXTURE_2D, 0);
  bloomSourceWidth_ = width;
  bloomSourceHeight_ = height;
  bloomWidth_ = bloomW;
  bloomHeight_ = bloomH;
  bloomResourcesReady_ = true;
  return true;
}

void Renderer::CompositeBloom() {
  if ((!bloomEnabledThisFrame_ && !screenOutlineEnabledThisFrame_) || !bloomResourcesReady_ || quadMesh_.vao.Get() == 0) {
    return;
  }

  const bool depthTestWasEnabled = glIsEnabled(GL_DEPTH_TEST) == GL_TRUE;
  backend_->SetDepthTestEnabled(false);
  glDepthMask(GL_FALSE);
  backend_->SetBlendEnabled(false);

  unsigned int sourceTexture = bloomPingTexture_.Get();
  int loc = -1;
  if (bloomEnabledThisFrame_) {
    // Bright pass extract.
    glBindFramebuffer(GL_FRAMEBUFFER, bloomPingFramebuffer_);
    backend_->ResizeViewport(bloomWidth_, bloomHeight_);
    backend_->ClearFrame(glm::vec4{0.0F, 0.0F, 0.0F, 1.0F});
    backend_->UseProgram(bloomExtractShader_.program.Get());
    loc = backend_->GetUniformLocation(bloomExtractShader_.program.Get(), "uThreshold");
    backend_->SetUniform1f(loc, bloomThreshold_);
    loc = backend_->GetUniformLocation(bloomExtractShader_.program.Get(), "uSceneTex");
    backend_->SetUniform1i(loc, 0);
    backend_->SetActiveTextureUnit(0);
    backend_->BindTexture2D(sceneColorTexture_.Get());
    backend_->BindVertexArray(quadMesh_.vao.Get());
    backend_->DrawIndexedTriangles(quadMesh_.indexCount);
    // Separable gaussian blur ping-pong.
    sourceTexture = bloomPingTexture_.Get();
    const int totalBlurPasses = std::max(1, bloomBlurPasses_ * 2);
    for (int pass = 0; pass < totalBlurPasses; ++pass) {
      const bool horizontal = (pass % 2) == 0;
      glBindFramebuffer(GL_FRAMEBUFFER, horizontal ? bloomPongFramebuffer_ : bloomPingFramebuffer_);
      backend_->UseProgram(bloomBlurShader_.program.Get());
      int dirLoc = backend_->GetUniformLocation(bloomBlurShader_.program.Get(), "uDirection");
      backend_->SetUniformVec3(dirLoc, horizontal ? 1.0F : 0.0F, horizontal ? 0.0F : 1.0F, 0.0F);
      int texelLoc = backend_->GetUniformLocation(bloomBlurShader_.program.Get(), "uTexelSize");
      backend_->SetUniformVec3(texelLoc, 1.0F / std::max(1, bloomWidth_), 1.0F / std::max(1, bloomHeight_), 0.0F);
      int srcLoc = backend_->GetUniformLocation(bloomBlurShader_.program.Get(), "uSourceTex");
      backend_->SetUniform1i(srcLoc, 0);
      backend_->SetActiveTextureUnit(0);
      backend_->BindTexture2D(sourceTexture);
      backend_->DrawIndexedTriangles(quadMesh_.indexCount);
      sourceTexture = horizontal ? bloomPongTexture_.Get() : bloomPingTexture_.Get();
    }
  } else {
    glBindFramebuffer(GL_FRAMEBUFFER, bloomPingFramebuffer_);
    backend_->ResizeViewport(bloomWidth_, bloomHeight_);
    backend_->ClearFrame(glm::vec4{0.0F, 0.0F, 0.0F, 1.0F});
    sourceTexture = bloomPingTexture_.Get();
  }

  // Composite to default framebuffer with optional screen-space outline.
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  backend_->ResizeViewport(width_, height_);
  backend_->UseProgram(bloomCompositeShader_.program.Get());
  int sceneLoc = backend_->GetUniformLocation(bloomCompositeShader_.program.Get(), "uSceneTex");
  int bloomLoc = backend_->GetUniformLocation(bloomCompositeShader_.program.Get(), "uBloomTex");
  int depthLoc = backend_->GetUniformLocation(bloomCompositeShader_.program.Get(), "uSceneDepthTex");
  int strengthLoc = backend_->GetUniformLocation(bloomCompositeShader_.program.Get(), "uBloomStrength");
  int outlineEnabledLoc = backend_->GetUniformLocation(bloomCompositeShader_.program.Get(), "uOutlineEnabled");
  int outlineStrengthLoc = backend_->GetUniformLocation(bloomCompositeShader_.program.Get(), "uOutlineStrength");
  int outlineThicknessLoc = backend_->GetUniformLocation(bloomCompositeShader_.program.Get(), "uOutlineThicknessPx");
  int outlineDepthSensitivityLoc = backend_->GetUniformLocation(bloomCompositeShader_.program.Get(), "uOutlineDepthSensitivity");
  int outlineNormalSensitivityLoc = backend_->GetUniformLocation(bloomCompositeShader_.program.Get(), "uOutlineNormalSensitivity");
  int outlineColorLoc = backend_->GetUniformLocation(bloomCompositeShader_.program.Get(), "uOutlineColor");
  int texelSizeLoc = backend_->GetUniformLocation(bloomCompositeShader_.program.Get(), "uTexelSize");
  backend_->SetUniform1i(sceneLoc, 0);
  backend_->SetUniform1i(bloomLoc, 1);
  backend_->SetUniform1i(depthLoc, 2);
  backend_->SetUniform1f(strengthLoc, bloomEnabledThisFrame_ ? bloomStrength_ : 0.0F);
  backend_->SetUniform1i(outlineEnabledLoc, screenOutlineEnabledThisFrame_ ? 1 : 0);
  backend_->SetUniform1f(outlineStrengthLoc, screenOutlineStrength_);
  backend_->SetUniform1f(outlineThicknessLoc, screenOutlineThicknessPx_);
  backend_->SetUniform1f(outlineDepthSensitivityLoc, screenOutlineDepthSensitivity_);
  backend_->SetUniform1f(outlineNormalSensitivityLoc, screenOutlineNormalSensitivity_);
  backend_->SetUniformVec3(
      outlineColorLoc,
      screenOutlineColor_.r,
      screenOutlineColor_.g,
      screenOutlineColor_.b);
  backend_->SetUniformVec3(
      texelSizeLoc,
      1.0F / static_cast<float>(std::max(1, width_)),
      1.0F / static_cast<float>(std::max(1, height_)),
      0.0F);
  backend_->SetActiveTextureUnit(0);
  backend_->BindTexture2D(sceneColorTexture_.Get());
  backend_->SetActiveTextureUnit(1);
  backend_->BindTexture2D(sourceTexture);
  backend_->SetActiveTextureUnit(2);
  backend_->BindTexture2D(sceneDepthTexture_.Get());
  backend_->DrawIndexedTriangles(quadMesh_.indexCount);
  backend_->BindVertexArray(0);
  backend_->SetActiveTextureUnit(0);
  backend_->BindTexture2D(0);
  backend_->SetActiveTextureUnit(1);
  backend_->BindTexture2D(0);
  backend_->SetActiveTextureUnit(2);
  backend_->BindTexture2D(0);
  backend_->SetActiveTextureUnit(0);

  glDepthMask(GL_TRUE);
  backend_->SetDepthTestEnabled(depthTestWasEnabled);
}

void Renderer::Draw(GLFWwindow* window, const OrbitCamera& camera, const SnapshotBundle& bundle) {
  SnapshotBundleSimWorld world(bundle);
  Draw(window, camera, world);
}

void Renderer::Draw(GLFWwindow* /*window*/, const OrbitCamera& camera, const ISimWorld& world) {
  const auto fullFrameStart = std::chrono::steady_clock::now();
  const bool bloomRequested = ParseEnvBool("RENDER_BLOOM_ENABLED", true);
  const bool screenOutlineRequested = ParseEnvBool("RENDER_SCREEN_OUTLINE_ENABLED", true);
  bloomStrength_ = std::clamp(ParseEnvFloat("RENDER_BLOOM_STRENGTH", bloomStrength_), 0.0F, 2.0F);
  bloomThreshold_ = std::clamp(ParseEnvFloat("RENDER_BLOOM_THRESHOLD", bloomThreshold_), 0.0F, 2.0F);
  bloomBlurPasses_ = std::clamp(ParseEnvInt("RENDER_BLOOM_BLUR_PASSES", bloomBlurPasses_), 1, 6);
  screenOutlineStrength_ =
      std::clamp(ParseEnvFloat("RENDER_SCREEN_OUTLINE_STRENGTH", screenOutlineStrength_), 0.0F, 2.0F);
  screenOutlineThicknessPx_ =
      std::clamp(ParseEnvFloat("RENDER_SCREEN_OUTLINE_THICKNESS_PX", screenOutlineThicknessPx_), 0.5F, 4.0F);
  screenOutlineDepthSensitivity_ = std::clamp(
      ParseEnvFloat("RENDER_SCREEN_OUTLINE_DEPTH_SENSITIVITY", screenOutlineDepthSensitivity_),
      0.2F,
      4.0F);
  screenOutlineNormalSensitivity_ = std::clamp(
      ParseEnvFloat("RENDER_SCREEN_OUTLINE_NORMAL_SENSITIVITY", screenOutlineNormalSensitivity_),
      0.0F,
      4.0F);
  screenOutlineColor_.r = std::clamp(ParseEnvFloat("RENDER_SCREEN_OUTLINE_COLOR_R", screenOutlineColor_.r), 0.0F, 1.0F);
  screenOutlineColor_.g = std::clamp(ParseEnvFloat("RENDER_SCREEN_OUTLINE_COLOR_G", screenOutlineColor_.g), 0.0F, 1.0F);
  screenOutlineColor_.b = std::clamp(ParseEnvFloat("RENDER_SCREEN_OUTLINE_COLOR_B", screenOutlineColor_.b), 0.0F, 1.0F);
  bloomEnabledThisFrame_ = false;
  screenOutlineEnabledThisFrame_ = false;
  const bool postRequested = bloomRequested || screenOutlineRequested;
  if (postRequested && EnsureBloomResources(width_, height_)) {
    glBindFramebuffer(GL_FRAMEBUFFER, sceneFramebuffer_);
    bloomEnabledThisFrame_ = bloomRequested;
    screenOutlineEnabledThisFrame_ = screenOutlineRequested;
  } else {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    bloomEnabledThisFrame_ = false;
    screenOutlineEnabledThisFrame_ = false;
  }
  backend_->ResizeViewport(width_, height_);
  backend_->ClearFrame(glm::vec4{0.06F, 0.06F, 0.07F, 1.0F});
  const auto afterClear = std::chrono::steady_clock::now();

  SceneToggleState toggles;
  toggles.showCameraDebug = showCameraDebug;
  toggles.showTruthFuel = showTruthFuel;
  toggles.showAgeFilteredFuel = showAgeFilteredFuel;
  toggles.showFieldImage = showFieldImage;

  RenderSceneFrame sceneFrame;
  if (worldAdapter_ != nullptr) {
    sceneFrame = worldAdapter_->BuildFrame(world, toggles);
  }
  const auto afterWorldBuild = std::chrono::steady_clock::now();

  const float aspect = static_cast<float>(width_) / static_cast<float>(height_);
  const glm::mat4 view = camera.ViewMatrix();
  const glm::mat4 projection = camera.ProjectionMatrix(aspect);
  const glm::mat4 vp = projection * view;
  DrawEnvironment(vp, camera.Eye());
  const auto afterEnvironment = std::chrono::steady_clock::now();
  const EntityCullingStats cullingStats = ApplyRenderEntityHierarchyAndCulling(sceneFrame, vp);
  const auto afterCulling = std::chrono::steady_clock::now();
  RenderCommandBuffer commandBuffer = BuildRenderCommandBuffer(sceneFrame);
  const auto afterCommandBuild = std::chrono::steady_clock::now();
  if (!HasMeshInstanceCommands(commandBuffer)) {
    diagnostics_.MarkSceneReady();
  }

  // Keep diagnostics data available to render features for internal adaptive policies
  // even when debug UI is disabled.
  const DiagnosticsSnapshot* diag = &diagnostics_.Latest();
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
  diagnostics_.RecordCounter("entities.stage.hierarchy_ms", cullingStats.hierarchyMs);
  diagnostics_.RecordCounter("entities.stage.transform_apply_ms", cullingStats.transformApplyMs);
  diagnostics_.RecordCounter("entities.stage.bounds_ms", cullingStats.boundsBuildMs);
  diagnostics_.RecordCounter("entities.stage.visibility_ms", cullingStats.visibilityTestMs);
  diagnostics_.RecordCounter("commands.total", static_cast<double>(commandBuffer.size()));
  const auto& caps = backend_->Capabilities();
  ++gpuTimerFrameCounter_;
  const bool gpuTimersRequested = caps.supportsGpuTimers && ParseEnvBool("RENDER_GPU_TIMERS", false);
  const int gpuTimerPollEveryFrames = std::max(1, ParseEnvInt("RENDER_GPU_TIMER_POLL_EVERY_FRAMES", 4));
  const bool allowGpuTimerReadThisFrame = (gpuTimerFrameCounter_ % static_cast<std::uint64_t>(gpuTimerPollEveryFrames)) == 0ULL;
  const double gpuTimerReadSlowMsThreshold =
      std::max(0.1, static_cast<double>(ParseEnvInt("RENDER_GPU_TIMER_READ_SLOW_MS", 3)));
  const int gpuTimerReadSlowDisableSamples =
      std::max(2, ParseEnvInt("RENDER_GPU_TIMER_READ_SLOW_DISABLE_SAMPLES", 3));
  const bool gpuTimersSparseSampling = ParseEnvBool("RENDER_GPU_TIMERS_SPARSE", true);
  const int gpuTimerOverheadDisableSamples =
      std::max(2, ParseEnvInt("RENDER_GPU_TIMER_OVERHEAD_DISABLE_SAMPLES", 4));
  const double gpuTimerOverheadDisableMs =
      std::max(1.0, static_cast<double>(ParseEnvInt("RENDER_GPU_TIMER_OVERHEAD_DISABLE_MS", 8)));
  const bool gpuTimersEnabled = gpuTimersRequested && !gpuTimersRuntimeDisabled_;
  diagnostics_.RecordCounter("backend.max_texture_size", static_cast<double>(caps.maxTextureSize));
  diagnostics_.RecordCounter("backend.max_vertex_attribs", static_cast<double>(caps.maxVertexAttribs));
  diagnostics_.RecordCounter("backend.supports_gpu_timers", caps.supportsGpuTimers ? 1.0 : 0.0);
  diagnostics_.RecordCounter("backend.gpu_timers_enabled", gpuTimersEnabled ? 1.0 : 0.0);
  diagnostics_.RecordCounter("backend.gpu_timers_requested", gpuTimersRequested ? 1.0 : 0.0);
  diagnostics_.RecordCounter("backend.gpu_timers_runtime_disabled", gpuTimersRuntimeDisabled_ ? 1.0 : 0.0);
  diagnostics_.RecordCounter(
      "backend.gpu_timer_poll_every_frames",
      static_cast<double>(gpuTimerPollEveryFrames));
  diagnostics_.RecordCounter("backend.gpu_timers_sparse", gpuTimersSparseSampling ? 1.0 : 0.0);
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
  diagnostics_.RecordCounter("arena.detail_divisor", static_cast<double>(environmentDetailDivisor_));
  diagnostics_.RecordCounter(
      "renderer.stage.clear_ms",
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(afterClear - fullFrameStart).count());
  diagnostics_.RecordCounter(
      "renderer.stage.world_build_ms",
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(afterWorldBuild - afterClear).count());
  diagnostics_.RecordCounter(
      "renderer.stage.environment_ms",
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(afterEnvironment - afterWorldBuild).count());
  diagnostics_.RecordCounter(
      "renderer.stage.entity_cull_ms",
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(afterCulling - afterEnvironment).count());
  diagnostics_.RecordCounter(
      "renderer.stage.command_build_ms",
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(afterCommandBuild - afterCulling).count());
  diagnostics_.RecordCounter(
      "renderer.stage.pregraph_ms",
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(afterCommandBuild - fullFrameStart).count());
  double appStageDrawMs = 0.0;
  bool hasAppStageDrawMs = false;
  for (const auto& [name, value] : queuedDiagnosticCounters_) {
    if (name == "app.stage.draw_ms") {
      appStageDrawMs = value;
      hasAppStageDrawMs = true;
    }
    diagnostics_.RecordCounter(name, value);
  }
  queuedDiagnosticCounters_.clear();
  for (const auto& message : queuedDiagnosticMessages_) {
    diagnostics_.RecordMessage(message);
  }
  queuedDiagnosticMessages_.clear();
  const auto frameStart = std::chrono::steady_clock::now();

  MaybeHotReloadRenderFeatures();
  if (!gpuTimersEnabled && !gpuPassTimers_.empty()) {
    for (auto& [_, timer] : gpuPassTimers_) {
      for (unsigned int& queryId : timer.queryIds) {
        if (queryId != 0) {
          backend_->DestroyGpuTimerQuery(queryId);
          queryId = 0;
        }
      }
    }
    gpuPassTimers_.clear();
  }
  if (!gpuTimersEnabled) {
    gpuTimerOverheadSlowSamples_ = 0;
  }

  RenderGraph graph;
  const std::size_t passCount = renderFeatures_.size();
  const std::size_t sampledPassIndex =
      (passCount > 0) ? (gpuTimerFrameCounter_ % static_cast<std::uint64_t>(passCount)) : 0U;
  double cpuPassAccumMs = 0.0;
  for (std::size_t passIndex = 0; passIndex < renderFeatures_.size(); ++passIndex) {
    const auto& feature = renderFeatures_[passIndex];
    if (feature == nullptr) {
      continue;
    }

    const bool passGpuTimerEnabled = gpuTimersEnabled &&
                                     (!gpuTimersSparseSampling || passIndex == sampledPassIndex);
    IRenderFeature* rawFeature = feature.get();
    graph.AddPass({
        .name = rawFeature->Name(),
        .dependencies = rawFeature->Dependencies(),
        .consumesResources = rawFeature->Dependencies(),
        .producesResources = {rawFeature->Name()},
        .execute =
            [this,
             rawFeature,
             &context,
             &drawApi,
             &cpuPassAccumMs,
             passGpuTimerEnabled,
             allowGpuTimerReadThisFrame,
             gpuTimerReadSlowMsThreshold,
             gpuTimerReadSlowDisableSamples](RenderGraphContext& graphContext) {
              GpuPassTimerState* gpuTimer = nullptr;
              bool gpuQueryActive = false;
              if (passGpuTimerEnabled) {
                auto& timerState = gpuPassTimers_[rawFeature->Name()];
                if (timerState.queryIds[0] == 0) {
                  timerState.queryIds[0] = backend_->CreateGpuTimerQuery();
                }
                if (timerState.queryIds[1] == 0) {
                  timerState.queryIds[1] = backend_->CreateGpuTimerQuery();
                }
                gpuTimer = &timerState;
                const unsigned int readQuery = timerState.queryIds[timerState.readIndex];
                if (allowGpuTimerReadThisFrame && readQuery != 0) {
                  const auto readStart = std::chrono::steady_clock::now();
                  double gpuMs = 0.0;
                  const bool readOk = backend_->TryReadGpuTimerMilliseconds(readQuery, gpuMs);
                  const auto readEnd = std::chrono::steady_clock::now();
                  const double readMs =
                      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(readEnd - readStart).count();
                  diagnostics_.RecordCounter("backend.gpu_timer_read_ms", readMs);
                  if (readMs >= gpuTimerReadSlowMsThreshold) {
                    ++gpuTimerReadSlowSamples_;
                  } else {
                    gpuTimerReadSlowSamples_ = std::max(0, gpuTimerReadSlowSamples_ - 1);
                  }
                  if (gpuTimerReadSlowSamples_ >= gpuTimerReadSlowDisableSamples) {
                    gpuTimersRuntimeDisabled_ = true;
                    QueueDiagnosticMessage(
                        "gpu timers auto-disabled due slow readback (read_ms=" + std::to_string(readMs) +
                        ", threshold_ms=" + std::to_string(gpuTimerReadSlowMsThreshold) + ")");
                  }
                  if (readOk) {
                    diagnostics_.RecordGpuTime(rawFeature->Name(), gpuMs);
                  }
                }
                const unsigned int writeQuery = timerState.queryIds[timerState.writeIndex];
                gpuQueryActive = writeQuery != 0;
                if (gpuQueryActive) {
                  backend_->BeginGpuTimerQuery(writeQuery);
                }
              }

              const auto start = std::chrono::steady_clock::now();
              rawFeature->Render(context, drawApi);
              const auto end = std::chrono::steady_clock::now();
              if (gpuQueryActive) {
                backend_->EndGpuTimerQuery();
              }

              const double ms =
                  std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start).count();
              cpuPassAccumMs += ms;
              diagnostics_.RecordPassTime(rawFeature->Name(), ms);

              if (gpuTimer != nullptr) {
                std::swap(gpuTimer->readIndex, gpuTimer->writeIndex);
              }
              graphContext.MarkResourceAvailable(rawFeature->Name());
            },
    });
  }

  graph.Execute();
  DrainAssetTelemetry();
  double bloomStageMs = 0.0;
  if (bloomEnabledThisFrame_ || screenOutlineEnabledThisFrame_) {
    const auto bloomStart = std::chrono::steady_clock::now();
    CompositeBloom();
    const auto bloomEnd = std::chrono::steady_clock::now();
    bloomStageMs =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(bloomEnd - bloomStart).count();
  } else {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    backend_->ResizeViewport(width_, height_);
  }

  const auto frameEnd = std::chrono::steady_clock::now();
  const double rendergraphMs =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(frameEnd - frameStart).count();
  const double nonPassOverheadMs = std::max(0.0, rendergraphMs - cpuPassAccumMs);
  diagnostics_.RecordCounter("backend.gpu_timer_non_pass_overhead_ms", nonPassOverheadMs);
  if (gpuTimersEnabled) {
    if (nonPassOverheadMs >= gpuTimerOverheadDisableMs) {
      ++gpuTimerOverheadSlowSamples_;
    } else {
      gpuTimerOverheadSlowSamples_ = std::max(0, gpuTimerOverheadSlowSamples_ - 1);
    }
    if (gpuTimerOverheadSlowSamples_ >= gpuTimerOverheadDisableSamples) {
      gpuTimersRuntimeDisabled_ = true;
      gpuTimerOverheadSlowSamples_ = 0;
      QueueDiagnosticMessage(
          "gpu timers auto-disabled due high non-pass overhead (overhead_ms=" +
          std::to_string(nonPassOverheadMs) + ", threshold_ms=" +
          std::to_string(gpuTimerOverheadDisableMs) + ")");
    }
  }
  const double rendererFrameMs =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(frameEnd - fullFrameStart).count();
  const double frameMs =
      (hasAppStageDrawMs && appStageDrawMs > 0.0) ? appStageDrawMs : rendererFrameMs;
  diagnostics_.RecordCounter("renderer.frame_ms", rendererFrameMs);
  diagnostics_.RecordCounter("renderer.frame_effective_ms", frameMs);
  diagnostics_.RecordCounter(
      "renderer.stage.rendergraph_ms",
      rendergraphMs);
  diagnostics_.RecordCounter("renderer.stage.bloom_ms", bloomStageMs);
  diagnostics_.RecordCounter("renderer.bloom.enabled", bloomEnabledThisFrame_ ? 1.0 : 0.0);
  diagnostics_.RecordCounter("renderer.post.screen_outline_enabled", screenOutlineEnabledThisFrame_ ? 1.0 : 0.0);
  diagnostics_.RecordCounter("renderer.post.screen_outline_strength", static_cast<double>(screenOutlineStrength_));
  diagnostics_.RecordCounter("renderer.post.screen_outline_thickness_px", static_cast<double>(screenOutlineThicknessPx_));
  diagnostics_.RecordCounter("renderer.bloom.strength", static_cast<double>(bloomStrength_));
  diagnostics_.RecordCounter("renderer.bloom.threshold", static_cast<double>(bloomThreshold_));
  diagnostics_.RecordCounter("renderer.bloom.blur_passes", static_cast<double>(bloomBlurPasses_));
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
  if (!cfg_.hotReloadRenderFeatures) {
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

void Renderer::DrawEnvironment(const glm::mat4& vp, const glm::vec3& cameraWorldPosition) {
  if (!cfg_.showEnvironmentBackdrop) {
    return;
  }

  constexpr float kPi = 3.14159265359F;
  const DiagnosticsSnapshot& latestDiag = diagnostics_.Latest();
  const double appLoopMs = FindCounterValue(latestDiag, "app.loop_ms", 0.0);
  const double appSwapWaitMs = FindCounterValue(latestDiag, "app.swap_wait_avg_ms", 0.0);
  const bool arenaAutoQuality = ParseEnvBool("ARENA_AUTO_QUALITY", false);
  int detailDivisor = std::clamp(ParseEnvInt("ARENA_DETAIL_DIVISOR", 1), 1, 6);
  if (arenaAutoQuality) {
    if (appLoopMs > 80.0 || appSwapWaitMs > 40.0) {
      detailDivisor = std::max(detailDivisor, 3);
    } else if (appLoopMs > 35.0 || appSwapWaitMs > 20.0) {
      detailDivisor = std::max(detailDivisor, 2);
    }
  }
  environmentDetailDivisor_ = detailDivisor;
  const int segments = std::max(8, cfg_.environmentSegments / detailDivisor);
  const bool mediumDetail = detailDivisor <= 2;
  const bool highDetail = detailDivisor <= 1;
  const float ringPadding = std::max(1.0F, cfg_.environmentRadiusM);
  const float stadiumHeight = std::max(3.0F, cfg_.environmentHeightM);
  const float fieldHalfLength = fieldLength_ * 0.5F;
  const float fieldHalfWidth = fieldWidth_ * 0.5F;
  const float ringBaseRadius = std::max(fieldHalfLength, fieldHalfWidth) + ringPadding;
  const float outerRadius = ringBaseRadius + 9.0F;
  const float floorZ = fieldZ_ - 0.05F;
  const auto now = std::chrono::steady_clock::now();
  static const auto kEnvStartTime = now;
  const float envTimeS =
      std::chrono::duration_cast<std::chrono::duration<float>>(now - kEnvStartTime).count();

  const bool cullWasEnabled = glIsEnabled(GL_CULL_FACE) == GL_TRUE;
  const bool depthTestWasEnabled = glIsEnabled(GL_DEPTH_TEST) == GL_TRUE;
  glDisable(GL_CULL_FACE);

  // Sky shell first: depth disabled + no depth writes, so it never occludes gameplay.
  backend_->SetDepthTestEnabled(false);
  glDepthMask(GL_FALSE);
  if (skyShader_.program.Get() != 0 && sphereMesh_.vao.Get() != 0) {
    glm::mat4 skyModel(1.0F);
    skyModel = glm::translate(skyModel, cameraWorldPosition);
    skyModel = glm::scale(skyModel, glm::vec3{outerRadius * 5.6F});
    const glm::mat4 skyMvp = vp * skyModel;

    const unsigned int skyProgram = skyShader_.program.Get();
    backend_->UseProgram(skyProgram);
    const int mvpLoc = backend_->GetUniformLocation(skyProgram, "uMvp");
    const int zenithLoc = backend_->GetUniformLocation(skyProgram, "uSkyZenith");
    const int horizonLoc = backend_->GetUniformLocation(skyProgram, "uSkyHorizon");
    const int groundLoc = backend_->GetUniformLocation(skyProgram, "uSkyGround");
    const int glowLoc = backend_->GetUniformLocation(skyProgram, "uArenaGlow");
    const int timeLoc = backend_->GetUniformLocation(skyProgram, "uTime");
    const float skyBeat = 0.5F + 0.5F * std::sin(envTimeS * 0.21F);
    backend_->SetUniformMat4(mvpLoc, &skyMvp[0][0]);
    backend_->SetUniformVec3(zenithLoc, 0.05F + 0.02F * skyBeat, 0.09F + 0.03F * skyBeat, 0.20F + 0.04F * skyBeat);
    backend_->SetUniformVec3(horizonLoc, 0.20F + 0.10F * skyBeat, 0.40F + 0.10F * skyBeat, 0.66F + 0.10F * skyBeat);
    backend_->SetUniformVec3(groundLoc, 0.025F, 0.025F, 0.05F + 0.02F * skyBeat);
    backend_->SetUniformVec3(glowLoc, 0.08F + 0.05F * skyBeat, 0.20F + 0.08F * skyBeat, 0.38F + 0.06F * skyBeat);
    backend_->SetUniform1f(timeLoc, envTimeS);
    backend_->BindVertexArray(sphereMesh_.vao.Get());
    backend_->DrawIndexedTriangles(sphereMesh_.indexCount);
    backend_->BindVertexArray(0);
  } else {
    DrawSphere(
        vp,
        SpherePrimitive{
            .center = cameraWorldPosition,
            .radius = outerRadius * 5.6F,
            .color = glm::vec4{0.13F, 0.21F, 0.35F, 1.0F},
            .pass = RenderPass::Background,
        });
  }

  DrawBox(
      vp,
      BoxPrimitive{
          .center = glm::vec3{cameraWorldPosition.x, cameraWorldPosition.y, fieldZ_ + stadiumHeight * 0.65F},
          .size = glm::vec3{outerRadius * 8.0F, outerRadius * 8.0F, 0.8F},
          .yawDeg = 0.0F,
          .color = glm::vec4{0.09F, 0.14F, 0.24F, 1.0F},
          .pass = RenderPass::Background,
      });

  // Distant skyline silhouettes around the stadium shell.
  const int skylineSegments = std::max(8, segments * (highDetail ? 3 : (mediumDetail ? 2 : 1)));
  const float skylineRadius = ringBaseRadius + 13.5F;
  for (int i = 0; i < skylineSegments; ++i) {
    const float angle = (2.0F * kPi * static_cast<float>(i)) / static_cast<float>(skylineSegments);
    const float yawDeg = glm::degrees(angle + (kPi * 0.5F));
    const float cx = std::cos(angle) * skylineRadius;
    const float cy = std::sin(angle) * skylineRadius;
    const float hSeed = Hash01(static_cast<std::uint32_t>(i * 8191 + 97));
    const float wSeed = Hash01(static_cast<std::uint32_t>(i * 4567 + 33));
    const float height = 5.5F + hSeed * 8.5F;
    const float width = 0.9F + wSeed * 1.4F;
    DrawBox(
        vp,
        BoxPrimitive{
            .center = glm::vec3{cx, cy, fieldZ_ + height * 0.5F - 0.2F},
            .size = glm::vec3{width, 1.0F, height},
            .yawDeg = yawDeg,
            .color = glm::vec4{0.04F, 0.05F, 0.07F, 1.0F},
            .pass = RenderPass::Opaque,
        });
    if (highDetail && (i % 3) == 0) {
      const float wndPulse = 0.45F + 0.55F * std::sin(envTimeS * 0.8F + static_cast<float>(i));
      DrawBox(
          vp,
          BoxPrimitive{
              .center = glm::vec3{cx, cy, fieldZ_ + height * 0.72F},
              .size = glm::vec3{width * 0.7F, 0.12F, height * 0.08F},
              .yawDeg = yawDeg,
              .color = glm::vec4{0.20F * wndPulse, 0.40F * wndPulse, 0.72F * wndPulse, 1.0F},
              .pass = RenderPass::Opaque,
          });
    }
  }
  glDepthMask(GL_TRUE);
  backend_->SetDepthTestEnabled(depthTestWasEnabled);

  const BoxPrimitive groundApron{
      .center = glm::vec3{0.0F, 0.0F, floorZ - 0.25F},
      .size = glm::vec3{outerRadius * 2.3F, outerRadius * 2.3F, 0.5F},
      .yawDeg = 0.0F,
      .color = glm::vec4{0.05F, 0.06F, 0.07F, 1.0F},
      .pass = RenderPass::Opaque,
  };
  DrawBox(vp, groundApron);

  // Neon trim around the playable field footprint.
  const float trimPulse = 0.72F + 0.28F * std::sin(envTimeS * 1.9F);
  const float trimZ = fieldZ_ + 0.02F;
  const float trimH = 0.03F;
  const float trimT = 0.06F;
  DrawBox(
      vp,
      BoxPrimitive{
          .center = glm::vec3{0.0F, fieldHalfWidth + trimT, trimZ},
          .size = glm::vec3{fieldLength_ + 0.14F, trimT, trimH},
          .yawDeg = 0.0F,
          .color = glm::vec4{0.18F * trimPulse, 0.74F * trimPulse, 1.0F * trimPulse, 1.0F},
          .pass = RenderPass::Opaque,
      });
  DrawBox(
      vp,
      BoxPrimitive{
          .center = glm::vec3{0.0F, -fieldHalfWidth - trimT, trimZ},
          .size = glm::vec3{fieldLength_ + 0.14F, trimT, trimH},
          .yawDeg = 0.0F,
          .color = glm::vec4{0.18F * trimPulse, 0.74F * trimPulse, 1.0F * trimPulse, 1.0F},
          .pass = RenderPass::Opaque,
      });
  DrawBox(
      vp,
      BoxPrimitive{
          .center = glm::vec3{fieldHalfLength + trimT, 0.0F, trimZ},
          .size = glm::vec3{fieldWidth_ + 0.14F, trimT, trimH},
          .yawDeg = 90.0F,
          .color = glm::vec4{0.22F * trimPulse, 0.62F * trimPulse, 0.96F * trimPulse, 1.0F},
          .pass = RenderPass::Opaque,
      });
  DrawBox(
      vp,
      BoxPrimitive{
          .center = glm::vec3{-fieldHalfLength - trimT, 0.0F, trimZ},
          .size = glm::vec3{fieldWidth_ + 0.14F, trimT, trimH},
          .yawDeg = 90.0F,
          .color = glm::vec4{0.22F * trimPulse, 0.62F * trimPulse, 0.96F * trimPulse, 1.0F},
          .pass = RenderPass::Opaque,
      });

  const glm::vec4 lowerTierA{0.18F, 0.21F, 0.24F, 1.0F};
  const glm::vec4 lowerTierB{0.12F, 0.14F, 0.17F, 1.0F};
  const glm::vec4 upperTierA{0.14F, 0.17F, 0.21F, 1.0F};
  const glm::vec4 upperTierB{0.10F, 0.12F, 0.16F, 1.0F};

  auto drawTier = [&](const float radius, const float zBase, const float height, const float thickness, const glm::vec4& colA, const glm::vec4& colB) {
    const float arcLength = (2.0F * kPi * radius / static_cast<float>(segments)) * 0.95F;
    for (int i = 0; i < segments; ++i) {
      const float angle = (2.0F * kPi * static_cast<float>(i)) / static_cast<float>(segments);
      const float angleDeg = glm::degrees(angle + (kPi * 0.5F));
      const float cx = std::cos(angle) * radius;
      const float cy = std::sin(angle) * radius;
      const float noise = Hash01(static_cast<std::uint32_t>(i * 92821 + 17));
      const float wave = 0.5F + 0.5F * std::sin(angle * 2.0F + static_cast<float>(i % 2));
      const glm::vec4 color = LerpColor(colA, colB, wave * 0.55F + noise * 0.45F);
      DrawBox(
          vp,
          BoxPrimitive{
              .center = glm::vec3{cx, cy, zBase + (height * 0.5F)},
              .size = glm::vec3{arcLength, thickness, height},
              .yawDeg = angleDeg,
              .color = color,
              .pass = RenderPass::Opaque,
          });
    }
  };

  drawTier(ringBaseRadius, fieldZ_, stadiumHeight * 0.55F, 3.2F, lowerTierA, lowerTierB);
  if (mediumDetail) {
    drawTier(ringBaseRadius + 4.8F, fieldZ_ + stadiumHeight * 0.33F, stadiumHeight * 0.35F, 2.8F, upperTierA, upperTierB);
  }
  if (highDetail) {
    drawTier(
        ringBaseRadius + 8.2F,
        fieldZ_ + stadiumHeight * 0.15F,
        stadiumHeight * 0.92F,
        1.6F,
        glm::vec4{0.08F, 0.10F, 0.13F, 1.0F},
        glm::vec4{0.05F, 0.06F, 0.08F, 1.0F});
  }

  // Crowd light band: tiny emissive-ish seat cells for a fuller arena look.
  if (highDetail) {
    const int crowdSegments = segments * 2;
    const float crowdRadius = ringBaseRadius + 2.3F;
    const float crowdArcLength = std::max(0.20F, (2.0F * kPi * crowdRadius / static_cast<float>(crowdSegments)) * 0.78F);
    for (int row = 0; row < 3; ++row) {
      const float rowZ = fieldZ_ + stadiumHeight * (0.20F + 0.10F * static_cast<float>(row));
      const float rowRadius = crowdRadius + static_cast<float>(row) * 0.45F;
      for (int i = 0; i < crowdSegments; ++i) {
        const float angle = (2.0F * kPi * static_cast<float>(i)) / static_cast<float>(crowdSegments);
        const float angleDeg = glm::degrees(angle + (kPi * 0.5F));
        const float cx = std::cos(angle) * rowRadius;
        const float cy = std::sin(angle) * rowRadius;
        const float seed = Hash01(static_cast<std::uint32_t>(row * 1000 + i * 313 + 19));
        const float pulse = 0.5F + 0.5F * std::sin(envTimeS * 1.4F + seed * 6.2831853F);
        const glm::vec4 crowdCol = LerpColor(
            glm::vec4{0.07F, 0.08F, 0.10F, 1.0F},
            glm::vec4{0.33F, 0.56F, 0.92F, 1.0F},
            pulse * 0.7F);
        DrawBox(
            vp,
            BoxPrimitive{
                .center = glm::vec3{cx, cy, rowZ},
                .size = glm::vec3{crowdArcLength, 0.25F, 0.08F},
                .yawDeg = angleDeg,
                .color = crowdCol,
                .pass = RenderPass::Opaque,
            });
      }
    }
  }

  const float fasciaRadius = ringBaseRadius + 1.8F;
  const float fasciaZ = fieldZ_ + stadiumHeight * 0.36F;
  const float fasciaLength = std::max(1.8F, (2.0F * kPi * fasciaRadius / static_cast<float>(segments)) * 0.72F);
  for (int i = 0; i < segments; ++i) {
    const float angle = (2.0F * kPi * static_cast<float>(i)) / static_cast<float>(segments);
    const float angleDeg = glm::degrees(angle + (kPi * 0.5F));
    const float cx = std::cos(angle) * fasciaRadius;
    const float cy = std::sin(angle) * fasciaRadius;
    const float pulse = 0.5F + 0.5F * std::sin(angle * 3.0F + envTimeS * 0.8F);
    const float sparkle = Hash01(static_cast<std::uint32_t>(i * 11939 + 911));
    const float scan = 0.5F + 0.5F * std::sin(angle * 11.0F - envTimeS * 4.2F);
    const glm::vec4 panelColor = LerpColor(
        glm::vec4{0.11F, 0.34F, 0.62F, 1.0F},
        glm::vec4{0.06F, 0.14F, 0.24F, 1.0F},
        pulse * 0.5F + sparkle * 0.2F + scan * 0.3F);
    DrawBox(
        vp,
        BoxPrimitive{
            .center = glm::vec3{cx, cy, fasciaZ},
            .size = glm::vec3{fasciaLength, 0.45F, 0.9F},
            .yawDeg = angleDeg,
            .color = panelColor,
            .pass = RenderPass::Opaque,
        });
  }

  if (mediumDetail) {
    const float lightRingRadius = ringBaseRadius + 2.2F;
    const float lightRingZ = fieldZ_ + stadiumHeight * 0.80F;
    const float lightBarLength = std::max(2.0F, (2.0F * kPi * lightRingRadius / static_cast<float>(segments)) * 0.74F);
    for (int i = 0; i < segments; i += 2) {
      const float angle = (2.0F * kPi * static_cast<float>(i)) / static_cast<float>(segments);
      const float angleDeg = glm::degrees(angle + (kPi * 0.5F));
      const float cx = std::cos(angle) * lightRingRadius;
      const float cy = std::sin(angle) * lightRingRadius;
      const float glow = 0.62F + 0.38F * std::cos(angle * 4.0F + envTimeS * 1.25F);
      const float hueJitter = Hash01(static_cast<std::uint32_t>(i * 4099 + 73));
      DrawBox(
          vp,
          BoxPrimitive{
              .center = glm::vec3{cx, cy, lightRingZ},
              .size = glm::vec3{lightBarLength, 0.42F, 0.30F},
              .yawDeg = angleDeg,
              .color =
                  glm::vec4{
                      (0.55F + 0.32F * hueJitter) * glow,
                      (0.78F + 0.16F * (1.0F - hueJitter)) * glow,
                      0.98F * glow,
                      1.0F},
              .pass = RenderPass::Opaque,
          });
    }
  }

  // Secondary ribbon board near upper bowl with faster chase lights.
  if (highDetail) {
    const float ribbonRadius = ringBaseRadius + 4.4F;
    const float ribbonZ = fieldZ_ + stadiumHeight * 0.72F;
    const float ribbonLen = std::max(1.4F, (2.0F * kPi * ribbonRadius / static_cast<float>(segments)) * 0.90F);
    for (int i = 0; i < segments; ++i) {
      const float angle = (2.0F * kPi * static_cast<float>(i)) / static_cast<float>(segments);
      const float angleDeg = glm::degrees(angle + (kPi * 0.5F));
      const float cx = std::cos(angle) * ribbonRadius;
      const float cy = std::sin(angle) * ribbonRadius;
      const float chase = 0.5F + 0.5F * std::sin(envTimeS * 5.2F - static_cast<float>(i) * 0.9F);
      DrawBox(
          vp,
          BoxPrimitive{
              .center = glm::vec3{cx, cy, ribbonZ},
              .size = glm::vec3{ribbonLen, 0.24F, 0.42F},
              .yawDeg = angleDeg,
              .color = glm::vec4{0.10F + 0.30F * chase, 0.24F + 0.60F * chase, 0.42F + 0.58F * chase, 1.0F},
              .pass = RenderPass::Opaque,
          });
    }
  }

  if (mediumDetail) {
    const float beaconRadius = ringBaseRadius + 3.4F;
    for (int i = 0; i < segments; i += (highDetail ? 3 : 6)) {
      const float angle = (2.0F * kPi * static_cast<float>(i)) / static_cast<float>(segments);
      const float cx = std::cos(angle) * beaconRadius;
      const float cy = std::sin(angle) * beaconRadius;
      const float hue = Hash01(static_cast<std::uint32_t>(i * 731 + 5));
      const float beat = 0.72F + 0.28F * std::sin(envTimeS * 1.7F + static_cast<float>(i));
      DrawBox(
          vp,
          BoxPrimitive{
              .center = glm::vec3{cx, cy, fieldZ_ + stadiumHeight * 0.63F},
              .size = glm::vec3{0.24F, 0.24F, 1.45F},
              .yawDeg = 0.0F,
              .color = glm::vec4{0.08F, 0.10F, 0.12F, 1.0F},
              .pass = RenderPass::Opaque,
          });
      DrawBox(
          vp,
          BoxPrimitive{
              .center = glm::vec3{cx, cy, fieldZ_ + stadiumHeight * 0.63F + 0.78F},
              .size = glm::vec3{0.56F, 0.56F, 0.14F},
              .yawDeg = 0.0F,
              .color =
                  glm::vec4{
                      (0.45F + 0.35F * hue) * beat,
                      (0.56F + 0.26F * (1.0F - hue)) * beat,
                      0.95F * beat,
                      1.0F},
              .pass = RenderPass::Opaque,
          });
    }
  }

  // Team tunnel portals on the cardinal directions.
  const float tunnelRadius = ringBaseRadius - 0.9F;
  const float tunnelZ = fieldZ_ + 0.95F;
  for (int i = 0; i < 4; ++i) {
    const float angle = static_cast<float>(i) * (kPi * 0.5F);
    const float yawDeg = glm::degrees(angle + (kPi * 0.5F));
    const float cx = std::cos(angle) * tunnelRadius;
    const float cy = std::sin(angle) * tunnelRadius;
    DrawBox(
        vp,
        BoxPrimitive{
            .center = glm::vec3{cx, cy, tunnelZ},
            .size = glm::vec3{2.8F, 1.3F, 1.8F},
            .yawDeg = yawDeg,
            .color = glm::vec4{0.06F, 0.07F, 0.09F, 1.0F},
            .pass = RenderPass::Opaque,
        });
    DrawBox(
        vp,
        BoxPrimitive{
            .center = glm::vec3{cx, cy, tunnelZ + 0.2F},
            .size = glm::vec3{2.2F, 0.15F, 1.2F},
            .yawDeg = yawDeg,
            .color =
                glm::vec4{
                    0.18F + 0.10F * std::sin(envTimeS + static_cast<float>(i)),
                    0.62F + 0.08F * std::cos(envTimeS * 0.7F + static_cast<float>(i)),
                    0.94F,
                    1.0F},
            .pass = RenderPass::Opaque,
        });
  }

  const float towerOffset = ringBaseRadius + 6.0F;
  const float towerHeight = stadiumHeight + 5.0F;
  for (const float sx : {-1.0F, 1.0F}) {
    for (const float sy : {-1.0F, 1.0F}) {
      DrawBox(
          vp,
          BoxPrimitive{
              .center = glm::vec3{sx * towerOffset, sy * towerOffset, fieldZ_ + towerHeight * 0.5F},
              .size = glm::vec3{1.8F, 1.8F, towerHeight},
              .yawDeg = 0.0F,
              .color = glm::vec4{0.13F, 0.15F, 0.19F, 1.0F},
              .pass = RenderPass::Opaque,
          });
      DrawBox(
          vp,
          BoxPrimitive{
              .center = glm::vec3{sx * towerOffset, sy * towerOffset, fieldZ_ + towerHeight + 0.8F},
              .size = glm::vec3{3.6F, 3.6F, 1.2F},
              .yawDeg = 0.0F,
              .color = glm::vec4{0.10F, 0.18F, 0.27F, 1.0F},
              .pass = RenderPass::Opaque,
          });
    }
  }

  const float roofZ = fieldZ_ + stadiumHeight + 2.8F;
  for (int i = 0; i < segments; ++i) {
    const float angle = (2.0F * kPi * static_cast<float>(i)) / static_cast<float>(segments);
    const float angleDeg = glm::degrees(angle + (kPi * 0.5F));
    const float cx = std::cos(angle) * (ringBaseRadius + 5.8F);
    const float cy = std::sin(angle) * (ringBaseRadius + 5.8F);
    DrawBox(
        vp,
        BoxPrimitive{
            .center = glm::vec3{cx, cy, roofZ},
            .size = glm::vec3{2.4F, 0.36F, 0.36F},
            .yawDeg = angleDeg,
            .color = glm::vec4{0.19F, 0.21F, 0.24F, 1.0F},
            .pass = RenderPass::Opaque,
        });
  }

  // Upper halo ring for stronger silhouette.
  if (mediumDetail) {
    const float haloRadius = ringBaseRadius + 7.4F;
    const float haloLength = std::max(1.0F, (2.0F * kPi * haloRadius / static_cast<float>(segments)) * 0.9F);
    for (int i = 0; i < segments; ++i) {
      const float angle = (2.0F * kPi * static_cast<float>(i)) / static_cast<float>(segments);
      const float angleDeg = glm::degrees(angle + (kPi * 0.5F));
      const float cx = std::cos(angle) * haloRadius;
      const float cy = std::sin(angle) * haloRadius;
      const float hue = Hash01(static_cast<std::uint32_t>(i * 827 + 11));
      const float sweep = 0.55F + 0.45F * std::sin(envTimeS * 1.6F + angle * 4.0F);
      DrawBox(
          vp,
          BoxPrimitive{
              .center = glm::vec3{cx, cy, roofZ + 0.95F},
              .size = glm::vec3{haloLength, 0.22F, 0.18F},
              .yawDeg = angleDeg,
              .color =
                  glm::vec4{
                      (0.24F + hue * 0.36F) * sweep,
                      (0.50F + hue * 0.24F) * sweep,
                      0.98F * sweep,
                      1.0F},
              .pass = RenderPass::Opaque,
          });
    }
  }

  // Light beams from upper rig toward the field center.
  if (highDetail) {
    const float beamRadius = ringBaseRadius + 5.0F;
    const int beamCount = std::max(12, segments);
    for (int i = 0; i < beamCount; ++i) {
      const float angle = (2.0F * kPi * static_cast<float>(i)) / static_cast<float>(beamCount);
      const float angleDeg = glm::degrees(angle + (kPi * 0.5F));
      const float cx = std::cos(angle) * beamRadius;
      const float cy = std::sin(angle) * beamRadius;
      const float glow = 0.45F + 0.55F * std::max(0.0F, std::cos(envTimeS * 1.25F + angle * 3.0F));
      DrawBox(
          vp,
          BoxPrimitive{
              .center = glm::vec3{cx * 0.78F, cy * 0.78F, fieldZ_ + stadiumHeight * 0.56F},
              .size = glm::vec3{0.16F, beamRadius * 0.20F, stadiumHeight * 0.72F},
              .yawDeg = angleDeg,
              .color = glm::vec4{0.35F * glow, 0.62F * glow, 1.0F * glow, 0.30F},
              .pass = RenderPass::Transparent,
          });
    }
  }

  // Suspended camera drones around scoreboard for extra motion cues.
  if (highDetail) {
    const float droneOrbitR = 4.4F;
    for (int i = 0; i < 6; ++i) {
      const float phase = envTimeS * 0.55F + static_cast<float>(i) * (2.0F * kPi / 6.0F);
      const float cx = std::cos(phase) * droneOrbitR;
      const float cy = std::sin(phase) * droneOrbitR;
      const float cz = fieldZ_ + stadiumHeight * (0.86F + 0.03F * std::sin(phase * 1.8F));
      DrawBox(
          vp,
          BoxPrimitive{
              .center = glm::vec3{cx, cy, cz},
              .size = glm::vec3{0.28F, 0.28F, 0.16F},
              .yawDeg = glm::degrees(phase),
              .color = glm::vec4{0.10F, 0.12F, 0.16F, 1.0F},
              .pass = RenderPass::Opaque,
          });
      DrawBox(
          vp,
          BoxPrimitive{
              .center = glm::vec3{cx, cy, cz - 0.12F},
              .size = glm::vec3{0.42F, 0.06F, 0.04F},
              .yawDeg = glm::degrees(phase + kPi * 0.5F),
              .color = glm::vec4{0.35F, 0.72F, 1.0F, 1.0F},
              .pass = RenderPass::Opaque,
          });
    }
  }

  // Rotating hologram rings above center for broadcast-style flair.
  if (highDetail) {
    const int holoSegments = std::max(24, segments * 2);
    for (int layer = 0; layer < 3; ++layer) {
      const float baseRadius = 1.8F + static_cast<float>(layer) * 0.75F;
      const float z = fieldZ_ + stadiumHeight * (0.70F + 0.05F * static_cast<float>(layer));
      const float rot = envTimeS * (0.7F + 0.25F * static_cast<float>(layer));
      const float len = std::max(0.10F, (2.0F * kPi * baseRadius / static_cast<float>(holoSegments)) * 0.72F);
      for (int i = 0; i < holoSegments; i += 2) {
        const float angle = (2.0F * kPi * static_cast<float>(i)) / static_cast<float>(holoSegments) + rot;
        const float angleDeg = glm::degrees(angle + (kPi * 0.5F));
        const float cx = std::cos(angle) * baseRadius;
        const float cy = std::sin(angle) * baseRadius;
        const float beat = 0.5F + 0.5F * std::sin(envTimeS * 4.5F + static_cast<float>(i));
        DrawBox(
            vp,
            BoxPrimitive{
                .center = glm::vec3{cx, cy, z},
                .size = glm::vec3{len, 0.06F, 0.035F},
                .yawDeg = angleDeg,
                .color =
                    glm::vec4{
                        (0.18F + 0.18F * static_cast<float>(layer)) * beat,
                        (0.55F + 0.14F * static_cast<float>(layer)) * beat,
                        0.98F * beat,
                        0.70F},
                .pass = RenderPass::Transparent,
            });
      }
    }
  }

  DrawBox(
      vp,
      BoxPrimitive{
          .center = glm::vec3{0.0F, 0.0F, roofZ - 0.4F},
          .size = glm::vec3{fieldLength_ * 0.95F, 0.28F, 0.28F},
          .yawDeg = 0.0F,
          .color = glm::vec4{0.25F, 0.27F, 0.31F, 1.0F},
          .pass = RenderPass::Opaque,
      });
  DrawBox(
      vp,
      BoxPrimitive{
          .center = glm::vec3{0.0F, 0.0F, roofZ - 0.4F},
          .size = glm::vec3{fieldWidth_ * 0.95F, 0.28F, 0.28F},
          .yawDeg = 90.0F,
          .color = glm::vec4{0.25F, 0.27F, 0.31F, 1.0F},
          .pass = RenderPass::Opaque,
      });

  DrawBox(
      vp,
      BoxPrimitive{
          .center = glm::vec3{0.0F, 0.0F, fieldZ_ + stadiumHeight * 0.86F},
          .size = glm::vec3{5.6F, 0.6F, 2.1F},
          .yawDeg = 0.0F,
          .color = glm::vec4{0.08F, 0.10F, 0.12F, 1.0F},
          .pass = RenderPass::Opaque,
      });
  DrawBox(
      vp,
      BoxPrimitive{
          .center = glm::vec3{0.0F, 0.12F, fieldZ_ + stadiumHeight * 0.86F},
          .size = glm::vec3{5.0F, 0.18F, 1.5F},
          .yawDeg = 0.0F,
          .color = glm::vec4{0.22F, 0.64F, 0.97F, 1.0F},
          .pass = RenderPass::Opaque,
      });

  if (cullWasEnabled) {
    glEnable(GL_CULL_FACE);
  } else {
    glDisable(GL_CULL_FACE);
  }
  backend_->SetDepthTestEnabled(depthTestWasEnabled);
  glDepthMask(GL_TRUE);
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
}

void Renderer::DrawOverlay(const int width, const int height, const std::vector<OverlayLine>& lines) {
  backend_->SetDepthTestEnabled(false);

  constexpr float scale = 2.0F;
  constexpr float lineStep = 8.0F * scale;
  constexpr float glyphHeight = 7.0F * scale;
  std::vector<OverlayLine> widgets = lines;

  // Always show a CAD loading bar while CAD assets are still being prepared/uploaded.
  const DiagnosticsSnapshot& diagSnapshot = diagnostics_.Latest();
  const double cadLoadsActive = FindCounterValue(diagSnapshot, "cad.loads.active", 0.0);
  const double cadUploadsPending = FindCounterValue(diagSnapshot, "cad.uploads.pending", 0.0);
  const double cadProgressPctRaw = FindCounterValue(diagSnapshot, "cad.loads.progress_pct", 0.0);
  const bool showCadLoadingBar = (cadLoadsActive > 0.5) || (cadUploadsPending > 0.5);
  const double cadProgressPct = std::clamp(cadProgressPctRaw, 0.0, 100.0);
  const int stageCode =
      static_cast<int>(std::round(FindCounterValue(diagSnapshot, "cad.loads.stage_code", 0.0)));
  const char* stageLabel = CadLoadStageLabelFromCode(stageCode);

  const double drawFps = FindCounterValue(diagSnapshot, "app.draw_fps", 0.0);
  const double loopFps = FindCounterValue(diagSnapshot, "app.loop_fps", 0.0);
  const double presentFps = FindCounterValue(diagSnapshot, "app.present_fps", 0.0);
  const double presentStall = FindCounterValue(diagSnapshot, "app.present_stall", 0.0);
  double displayedFps = 0.0;
  if (presentStall > 0.5) {
    displayedFps = (loopFps > 1e-3) ? loopFps : ((presentFps > 1e-3) ? presentFps : drawFps);
  } else {
    displayedFps = (drawFps > 1e-3) ? drawFps : ((loopFps > 1e-3) ? loopFps : presentFps);
  }
  if (displayedFps > 1e-3) {
    std::ostringstream fpsText;
    fpsText << std::fixed << std::setprecision(1) << "FPS: " << displayedFps;
    widgets.push_back(
        {.text = fpsText.str(),
         .color = glm::vec4{0.92F, 0.92F, 0.92F, 0.90F},
         .anchor = OverlayAnchor::TopRight,
         .marginX = 10.0F,
         .marginY = 12.0F});
  } else if (smoothedFrameMsInitialized_ && smoothedFrameMs_ > 1e-6) {
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

  if (showCadLoadingBar) {
    constexpr float barScale = 2.1F;
    constexpr float stageScale = 1.7F;
    constexpr float marginX = 10.0F;
    constexpr float marginY = 8.0F;
    constexpr float lineGap = 4.0F;

    auto makeLoadingLine = [&](const int barChars) {
      const int filled = std::clamp(
          static_cast<int>(std::round((cadProgressPct / 100.0) * static_cast<double>(barChars))),
          0,
          barChars);
      std::string bar(static_cast<std::size_t>(filled), '#');
      bar.append(static_cast<std::size_t>(barChars - filled), '-');

      std::ostringstream loadingText;
      loadingText << "CAD LOAD [" << bar << "] " << std::fixed << std::setprecision(0) << cadProgressPct << "%";
      return loadingText.str();
    };

    int barChars = 30;
    std::string loadingLine = makeLoadingLine(barChars);
    const float availableWidth = std::max(64.0F, static_cast<float>(width) - (2.0F * marginX));
    while (barChars > 8 && TextWidthPixels(loadingLine, barScale) > availableWidth) {
      --barChars;
      loadingLine = makeLoadingLine(barChars);
    }

    const double stageProgressPct = std::clamp(
        FindCounterValue(diagSnapshot, "cad.loads.stage_progress_pct", 0.0),
        0.0,
        100.0);
    const double stageElapsedS = std::max(0.0, FindCounterValue(diagSnapshot, "cad.loads.stage_elapsed_s", 0.0));

    std::ostringstream stageLineStream;
    stageLineStream << "stage: " << stageLabel
                    << " " << std::fixed << std::setprecision(1) << stageElapsedS << "s"
                    << " (" << std::setprecision(0) << stageProgressPct << "%)"
                    << " | L:" << static_cast<int>(std::round(cadLoadsActive))
                    << " U:" << static_cast<int>(std::round(cadUploadsPending));
    const std::string stageLine = stageLineStream.str();

    const float maxTextWidth = std::max(
        TextWidthPixels(loadingLine, barScale),
        TextWidthPixels(stageLine, stageScale));
    const float x = std::max(2.0F, static_cast<float>(width) - marginX - maxTextWidth);

    const float stageGlyphHeight = 7.0F * stageScale;
    const float barGlyphHeight = 7.0F * barScale;
    float stageY = marginY + stageGlyphHeight;
    float barY = stageY + lineGap + barGlyphHeight;

    const float maxY = static_cast<float>(height) - 2.0F;
    if (barY > maxY) {
      const float overflow = barY - maxY;
      stageY -= overflow;
      barY -= overflow;
    }
    const float minStageY = 2.0F + stageGlyphHeight;
    if (stageY < minStageY) {
      const float lift = minStageY - stageY;
      stageY += lift;
      barY += lift;
    }
    stageY = std::clamp(stageY, minStageY, maxY);
    barY = std::clamp(barY, 2.0F + barGlyphHeight, maxY);

    DrawText2D(x, barY, barScale, loadingLine, glm::vec4{0.30F, 0.84F, 1.00F, 0.98F});
    DrawText2D(x, stageY, stageScale, stageLine, glm::vec4{0.86F, 0.92F, 1.00F, 0.94F});
  }

  backend_->SetDepthTestEnabled(true);
}

}  // namespace repulsor3d
