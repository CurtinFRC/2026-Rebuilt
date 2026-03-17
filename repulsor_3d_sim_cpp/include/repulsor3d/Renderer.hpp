#pragma once

#include <array>
#include <memory>
#include <string>
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
#include "repulsor3d/render/assets/SceneAssetResolver.hpp"
#include "repulsor3d/render/backend/GlHandles.hpp"
#include "repulsor3d/render/backend/RenderBackend.hpp"
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
  IRenderBackend& GetRenderBackend();
  IGeometryProvider& GetGeometryProvider();
  ISceneAssetResolver& GetAssetResolver();
  const DiagnosticsSnapshot& LatestDiagnostics() const;

  bool showCameraDebug = true;
  bool showTruthFuel = true;
  bool showAgeFilteredFuel = false;
  bool showFieldImage = true;

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

  bool CreateShaders();
  bool CreateMeshes();
  bool CreateFieldTexture();

  void DestroyShader(Shader& shader) const;
  void DestroyMesh(Mesh& mesh) const;

  unsigned int CompileShader(unsigned int type, const char* source) const;
  bool LinkShader(Shader& shader, unsigned int vs, unsigned int fs) const;

  void DrawGrid(const glm::mat4& vp);
  void DrawAxes(const glm::mat4& vp);
  void DrawFieldImage(const glm::mat4& vp);

  void DrawBox(const glm::mat4& vp, const BoxPrimitive& primitive);
  void DrawSphere(const glm::mat4& vp, const SpherePrimitive& primitive);
  void DrawLinePrimitives(const glm::mat4& vp, const std::vector<LinePrimitive>& lines);
  void DrawLineList(const glm::mat4& vp, const std::vector<VertexPC>& vertices, float width = 1.0F);

  void DrawOverlay(int width, int height, const std::vector<OverlayLine>& lines);
  void DrawText2D(float x, float y, float scale, const std::string& text, const glm::vec4& color);

  static float TextWidthPixels(const std::string& text, float scale);
  static std::string ToUpperAscii(const std::string& s);
  static std::array<std::string, 7> Glyph(char c);

  ViewerConfig cfg_;
  int width_ = 1;
  int height_ = 1;

  std::unique_ptr<IRenderWorldAdapter> worldAdapter_;
  std::vector<std::unique_ptr<IRenderFeature>> renderFeatures_;
  std::unique_ptr<IRenderBackend> backend_;
  std::unique_ptr<ISceneAssetResolver> assetResolver_;
  std::unique_ptr<IGeometryProvider> geometryProvider_;
  DiagnosticsService diagnostics_;

  Shader solidShader_;
  Shader lineShader_;
  Shader texturedShader_;

  Mesh cubeMesh_;
  Mesh sphereMesh_;
  Mesh quadMesh_;

  GlVertexArrayHandle dynamicLineVao_;
  GlBufferHandle dynamicLineVbo_;

  GlVertexArrayHandle textVao_;
  GlBufferHandle textVbo_;

  GlTextureHandle fieldTexture_;

  float fieldLength_ = 0.0F;
  float fieldWidth_ = 0.0F;
  float fieldZ_ = 0.0F;

  bool smoothedFrameMsInitialized_ = false;
  double smoothedFrameMs_ = 0.0;
};

}  // namespace repulsor3d
