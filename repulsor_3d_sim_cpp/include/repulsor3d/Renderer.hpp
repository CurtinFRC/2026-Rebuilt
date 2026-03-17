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
#include "repulsor3d/Model.hpp"
#include "repulsor3d/render/SceneFrame.hpp"
#include "repulsor3d/render/SceneModelBuilder.hpp"

struct GLFWwindow;

namespace repulsor3d {

class Renderer {
 public:
  explicit Renderer(const ViewerConfig& cfg, std::unique_ptr<ISceneModelBuilder> sceneBuilder = nullptr);
  ~Renderer();

  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;

  bool Initialize();
  void Resize(int w, int h);
  void Draw(GLFWwindow* window, const OrbitCamera& camera, const SnapshotBundle& bundle);

  void SetSceneModelBuilder(std::unique_ptr<ISceneModelBuilder> sceneBuilder);

  bool showCameraDebug = true;
  bool showTruthFuel = true;
  bool showAgeFilteredFuel = false;
  bool showFieldImage = true;

 private:
  struct Mesh {
    unsigned int vao = 0;
    unsigned int vbo = 0;
    unsigned int ebo = 0;
    int indexCount = 0;
  };

  struct Shader {
    unsigned int id = 0;
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
  void DrawLineList(const glm::mat4& vp, const std::vector<VertexPC>& vertices, float width = 1.0F);

  void DrawOverlay(int width, int height, const std::vector<OverlayLine>& lines);
  void DrawText2D(float x, float y, float scale, const std::string& text, const glm::vec4& color);

  static std::string ToUpperAscii(const std::string& s);
  static std::array<std::string, 7> Glyph(char c);

  ViewerConfig cfg_;
  int width_ = 1;
  int height_ = 1;

  std::unique_ptr<ISceneModelBuilder> sceneBuilder_;

  Shader solidShader_;
  Shader lineShader_;
  Shader texturedShader_;

  Mesh cubeMesh_;
  Mesh sphereMesh_;
  Mesh quadMesh_;

  unsigned int dynamicLineVao_ = 0;
  unsigned int dynamicLineVbo_ = 0;

  unsigned int textVao_ = 0;
  unsigned int textVbo_ = 0;

  unsigned int fieldTexture_ = 0;

  float fieldLength_ = 0.0F;
  float fieldWidth_ = 0.0F;
  float fieldZ_ = 0.0F;
};

}  // namespace repulsor3d
