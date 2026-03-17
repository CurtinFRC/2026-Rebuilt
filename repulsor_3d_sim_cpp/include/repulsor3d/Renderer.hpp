#pragma once

#include <array>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "repulsor3d/Camera.hpp"
#include "repulsor3d/Config.hpp"
#include "repulsor3d/Model.hpp"

struct GLFWwindow;

namespace repulsor3d {

class Renderer {
 public:
  explicit Renderer(const ViewerConfig& cfg);
  ~Renderer();

  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;

  bool Initialize();
  void Resize(int w, int h);

  void Draw(GLFWwindow* window, const OrbitCamera& camera, const SnapshotBundle& bundle);

  bool showCameraDebug = true;
  bool showTruthFuel = true;
  bool showAgeFilteredFuel = false;
  bool showFieldImage = true;

 public:
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

 private:

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
  void DrawFuel(const glm::mat4& vp, const WorldSnapshot& snap);
  void DrawTruthFuel(const glm::mat4& vp, const WorldSnapshot& snap);
  void DrawOtherRobots(const glm::mat4& vp, const WorldSnapshot& snap);
  void DrawRobotTargets(const glm::mat4& vp, const WorldSnapshot& snap);
  void DrawCameras(const glm::mat4& vp, const WorldSnapshot& snap);

  void DrawBox(const glm::mat4& vp, const glm::vec3& center, const glm::vec3& size, float yawDeg,
               const glm::vec4& color);
  void DrawSphere(const glm::mat4& vp, const glm::vec3& center, float radius, const glm::vec4& color);
  void DrawLineList(const glm::mat4& vp, const std::vector<VertexPC>& vertices, float width = 1.0F);

  void DrawOverlay(int width, int height, const SnapshotBundle& bundle);
  void DrawText2D(float x, float y, float scale, const std::string& text, const glm::vec4& color);

  static std::string ToUpperAscii(const std::string& s);
  static std::array<std::string, 7> Glyph(char c);

  ViewerConfig cfg_;
  int width_ = 1;
  int height_ = 1;

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

  float fuelRadius_ = 0.0F;
  float obsSide_ = 0.0F;
  float robotL_ = 0.0F;
  float robotW_ = 0.0F;
  float robotH_ = 0.0F;

  glm::vec4 colFuel_{1.0F, 0.95F, 0.15F, 0.95F};
  glm::vec4 colTruthFuel_{0.15F, 0.95F, 0.35F, 0.70F};
  glm::vec4 colOther_{1.0F, 0.15F, 0.15F, 0.85F};
  glm::vec4 colUs_{1.0F, 0.27F, 0.0F, 0.75F};
  glm::vec4 colActive_{0.15F, 0.9F, 0.3F, 0.9F};
  glm::vec4 colChosen_{0.15F, 0.3F, 0.9F, 0.9F};
  glm::vec4 colFinal_{0.9F, 0.3F, 0.15F, 0.9F};
  glm::vec4 colHeading_{1.0F, 0.1F, 0.1F, 0.9F};
  glm::vec4 colCam_{0.20F, 0.75F, 1.0F, 0.85F};
  glm::vec4 colCamFov_{0.20F, 0.75F, 1.0F, 0.55F};
  glm::vec4 colCamRayOk_{0.20F, 0.95F, 0.35F, 0.55F};
  glm::vec4 colCamRayBad_{1.00F, 0.25F, 0.25F, 0.40F};

  std::unordered_map<std::string, double> fuelLastSeen_;
  std::unordered_map<std::string, FieldVisionObject> fuelCache_;
};

}  // namespace repulsor3d
