#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "repulsor3d/Camera.hpp"
#include "repulsor3d/Config.hpp"
#include "repulsor3d/Diagnostics.hpp"
#include "repulsor3d/Model.hpp"
#include "repulsor3d/render/RenderFeature.hpp"
#include "repulsor3d/render/RenderFeaturePlugin.hpp"
#include "repulsor3d/render/assets/SceneAssetResolver.hpp"
#include "repulsor3d/render/backend/GlHandles.hpp"
#include "repulsor3d/render/backend/RenderBackend.hpp"
#include "repulsor3d/render/resources/ResourceLifetimeManager.hpp"
#include "repulsor3d/render/geometry/GeometryProvider.hpp"
#include "repulsor3d/render/SceneFrame.hpp"
#include "repulsor3d/render/SceneModelBuilder.hpp"
#include "repulsor3d/render/RenderWorldAdapter.hpp"
#include "repulsor3d/sim/SimWorld.hpp"

struct GLFWwindow;

namespace repulsor3d {

class Renderer {
 public:
  explicit Renderer(const ViewerConfig& cfg, std::unique_ptr<IRenderWorldAdapter> worldAdapter = nullptr);
  ~Renderer();

  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;

  bool Initialize();
  void Resize(int w, int h);
  void Draw(GLFWwindow* window, const OrbitCamera& camera, const ISimWorld& world);
  void Draw(GLFWwindow* window, const OrbitCamera& camera, const SnapshotBundle& bundle);

  void SetRenderWorldAdapter(std::unique_ptr<IRenderWorldAdapter> worldAdapter);
  void SetSceneModelBuilder(std::unique_ptr<ISceneModelBuilder> sceneBuilder);
  void SetRenderFeatures(std::vector<std::unique_ptr<IRenderFeature>> renderFeatures);
  void ApplyRuntimeConfig(const ViewerConfig& cfg);
  void QueueDiagnosticMessage(std::string message);
  void QueueDiagnosticCounter(std::string name, double value);
  IRenderBackend& GetRenderBackend();
  IGeometryProvider& GetGeometryProvider();
  ISceneAssetResolver& GetAssetResolver();
  const DiagnosticsSnapshot& LatestDiagnostics() const;

  bool showCameraDebug = true;
  bool showTruthFuel = true;
  bool showAgeFilteredFuel = false;
  bool showFieldImage = true;
  bool showDebugPanel = true;
  bool showDebugCounters = true;
  bool showDebugCpu = true;
  bool showDebugGpu = true;
  bool showDebugAssets = true;

 private:
  friend struct RendererDrawApi;

  struct Mesh {
    GlVertexArrayHandle vao;
    GlBufferHandle vbo;
    GlBufferHandle ebo;
    int indexCount = 0;
  };

  struct Shader {
    GlProgramHandle program;
  };

  struct VertexPC {
    glm::vec3 pos;
    glm::vec4 color;
  };

  struct VertexP {
    glm::vec3 pos;
  };

  struct VertexPT {
    glm::vec3 pos;
    glm::vec2 uv;
  };

  struct GpuPassTimerState {
    std::array<unsigned int, 2> queryIds{0, 0};
    int readIndex = 0;
    int writeIndex = 1;
  };

  bool CreateShaders();
  bool CreateMeshes();
  bool CreateFieldTexture();
  bool EnsureBloomResources(int width, int height);
  void DestroyBloomResources();
  void CompositeBloom();

  void DestroyShader(Shader& shader) const;
  void DestroyMesh(Mesh& mesh);

  unsigned int CompileShader(unsigned int type, const char* source) const;
  bool LinkShader(Shader& shader, unsigned int vs, unsigned int fs) const;

  void DrawGrid(const glm::mat4& vp);
  void DrawAxes(const glm::mat4& vp);
  void DrawFieldImage(const glm::mat4& vp);
  void DrawEnvironment(const glm::mat4& vp, const glm::vec3& cameraWorldPosition);

  void DrawBox(const glm::mat4& vp, const BoxPrimitive& primitive);
  void DrawSphere(const glm::mat4& vp, const SpherePrimitive& primitive);
  void DrawLinePrimitives(const glm::mat4& vp, const std::vector<LinePrimitive>& lines);
  void DrawLineList(const glm::mat4& vp, const std::vector<VertexPC>& vertices, float width = 1.0F);

  void DrawOverlay(int width, int height, const std::vector<OverlayLine>& lines);
  void DrawText2D(float x, float y, float scale, const std::string& text, const glm::vec4& color);
  void DrainAssetTelemetry();
  bool RebuildRenderFeatures(std::string reason);
  void MaybeHotReloadRenderFeatures();
  void CaptureRenderFeatureInputStamps();
  static bool QueryFileWriteTime(const std::string& path, std::filesystem::file_time_type& outWriteTime);

  static float TextWidthPixels(const std::string& text, float scale);
  static std::string ToUpperAscii(const std::string& s);
  static std::array<std::string, 7> Glyph(char c);

  ViewerConfig cfg_;
  int width_ = 1;
  int height_ = 1;

  std::unique_ptr<IRenderWorldAdapter> worldAdapter_;
  std::vector<std::unique_ptr<IRenderFeature>> renderFeatures_;
  std::unique_ptr<IRenderFeaturePlugin> renderFeaturePlugin_;
  std::unique_ptr<IRenderBackend> backend_;
  std::unique_ptr<ISceneAssetResolver> assetResolver_;
  std::unique_ptr<IGeometryProvider> geometryProvider_;
  ResourceLifetimeManager resourceManager_;
  DiagnosticsService diagnostics_;

  Shader solidShader_;
  Shader lineShader_;
  Shader texturedShader_;
  Shader skyShader_;
  Shader bloomExtractShader_;
  Shader bloomBlurShader_;
  Shader bloomCompositeShader_;

  Mesh cubeMesh_;
  Mesh sphereMesh_;
  Mesh quadMesh_;

  GlVertexArrayHandle dynamicLineVao_;
  GlBufferHandle dynamicLineVbo_;

  GlVertexArrayHandle textVao_;
  GlBufferHandle textVbo_;

  GlTextureHandle fieldTexture_;
  GlTextureHandle sceneColorTexture_;
  GlTextureHandle bloomPingTexture_;
  GlTextureHandle bloomPongTexture_;
  unsigned int sceneFramebuffer_ = 0;
  unsigned int sceneDepthStencilRbo_ = 0;
  unsigned int bloomPingFramebuffer_ = 0;
  unsigned int bloomPongFramebuffer_ = 0;
  int bloomSourceWidth_ = 0;
  int bloomSourceHeight_ = 0;
  int bloomWidth_ = 0;
  int bloomHeight_ = 0;
  int bloomDownsample_ = 2;
  bool bloomResourcesReady_ = false;
  bool bloomEnabledThisFrame_ = false;
  float bloomStrength_ = 0.14F;
  float bloomThreshold_ = 0.96F;
  int bloomBlurPasses_ = 2;

  float fieldLength_ = 0.0F;
  float fieldWidth_ = 0.0F;
  float fieldZ_ = 0.0F;

  bool smoothedFrameMsInitialized_ = false;
  double smoothedFrameMs_ = 0.0;
  std::unordered_map<std::string, GpuPassTimerState> gpuPassTimers_;
  bool gpuTimersRuntimeDisabled_ = false;
  int gpuTimerReadSlowSamples_ = 0;
  int gpuTimerOverheadSlowSamples_ = 0;
  std::uint64_t gpuTimerFrameCounter_ = 0;

  bool renderFeaturesInitialized_ = false;
  std::string renderPipelinePath_;
  std::filesystem::file_time_type renderPipelineWriteTime_{};
  bool renderPipelineWriteTimeKnown_ = false;
  std::string renderFeaturePluginPath_;
  std::filesystem::file_time_type renderFeaturePluginWriteTime_{};
  bool renderFeaturePluginWriteTimeKnown_ = false;
  std::chrono::steady_clock::time_point nextRenderFeatureReloadCheck_{};
  std::uint64_t renderFeatureReloadCount_ = 0;
  std::uint64_t renderFeatureReloadFailedCount_ = 0;
  std::string renderFeatureReloadLastError_;
  int environmentDetailDivisor_ = 1;
  std::vector<std::string> queuedDiagnosticMessages_;
  std::vector<std::pair<std::string, double>> queuedDiagnosticCounters_;
};

}  // namespace repulsor3d
