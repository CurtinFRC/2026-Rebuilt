#include "repulsor3d/Renderer.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <map>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>

namespace repulsor3d {
namespace {

constexpr float kPi = 3.14159265358979323846F;

template <typename TVertex>
bool UploadMesh(
    const std::vector<TVertex>& vertices,
    const std::vector<uint32_t>& indices,
    Renderer::Mesh& mesh,
    const bool withUv = false) {
  glGenVertexArrays(1, &mesh.vao);
  glGenBuffers(1, &mesh.vbo);
  glGenBuffers(1, &mesh.ebo);

  if (mesh.vao == 0 || mesh.vbo == 0 || mesh.ebo == 0) {
    return false;
  }

  glBindVertexArray(mesh.vao);

  glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
  glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(TVertex)), vertices.data(), GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indices.size() * sizeof(uint32_t)), indices.data(), GL_STATIC_DRAW);

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(TVertex), reinterpret_cast<void*>(0));

  if (withUv) {
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(TVertex), reinterpret_cast<void*>(offsetof(Renderer::VertexPT, uv)));
  }

  glBindVertexArray(0);
  mesh.indexCount = static_cast<int>(indices.size());
  return true;
}

std::string NormalizeType(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

}  // namespace

Renderer::Renderer(const ViewerConfig& cfg) : cfg_(cfg) {
  showCameraDebug = cfg.showCameraDebug;
  showTruthFuel = cfg.showTruthFuel;
  showAgeFilteredFuel = cfg.showAgeFilteredFuel;
  showFieldImage = cfg.showFieldImage;

  width_ = std::max(1, cfg.windowW);
  height_ = std::max(1, cfg.windowH);

  fieldLength_ = cfg.fieldLengthM;
  fieldWidth_ = cfg.fieldWidthM;
  fieldZ_ = cfg.fieldZM;

  fuelRadius_ = cfg.ballRadiusM;
  obsSide_ = cfg.obsBoxSideM;
  robotL_ = cfg.robotBoxLM;
  robotW_ = cfg.robotBoxWM;
  robotH_ = cfg.robotBoxHM;
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
    std::cerr << "Renderer: field texture missing or failed to load, continuing without it\n";
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

  const float aspect = static_cast<float>(width_) / static_cast<float>(height_);
  const glm::mat4 vp = camera.ProjectionMatrix(aspect) * camera.ViewMatrix();

  DrawGrid(vp);
  if (showFieldImage && fieldTexture_ != 0) {
    DrawFieldImage(vp);
  }
  DrawAxes(vp);

  const WorldSnapshot& snap = bundle.snapshot;
  if (showTruthFuel) {
    DrawTruthFuel(vp, snap);
  }
  DrawFuel(vp, snap);
  DrawOtherRobots(vp, snap);
  DrawRobotTargets(vp, snap);
  if (showCameraDebug) {
    DrawCameras(vp, snap);
  }

  DrawOverlay(width_, height_, bundle);
}

bool Renderer::CreateShaders() {
  constexpr const char* solidVs = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 uMvp;
void main() {
  gl_Position = uMvp * vec4(aPos, 1.0);
}
)";

  constexpr const char* solidFs = R"(
#version 330 core
uniform vec4 uColor;
out vec4 FragColor;
void main() {
  FragColor = uColor;
}
)";

  constexpr const char* lineVs = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;
uniform mat4 uMvp;
out vec4 vColor;
void main() {
  vColor = aColor;
  gl_Position = uMvp * vec4(aPos, 1.0);
}
)";

  constexpr const char* lineFs = R"(
#version 330 core
in vec4 vColor;
out vec4 FragColor;
void main() {
  FragColor = vColor;
}
)";

  constexpr const char* texVs = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aUv;
uniform mat4 uMvp;
out vec2 vUv;
void main() {
  vUv = aUv;
  gl_Position = uMvp * vec4(aPos, 1.0);
}
)";

  constexpr const char* texFs = R"(
#version 330 core
in vec2 vUv;
uniform sampler2D uTex;
uniform float uAlpha;
uniform int uFlipX;
uniform int uFlipY;
out vec4 FragColor;
void main() {
  vec2 uv = vUv;
  if (uFlipX == 1) {
    uv.x = 1.0 - uv.x;
  }
  if (uFlipY == 1) {
    uv.y = 1.0 - uv.y;
  }
  vec4 texColor = texture(uTex, uv);
  FragColor = vec4(texColor.rgb, texColor.a * uAlpha);
}
)";

  unsigned int vs = CompileShader(GL_VERTEX_SHADER, solidVs);
  unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, solidFs);
  if (vs == 0 || fs == 0 || !LinkShader(solidShader_, vs, fs)) {
    return false;
  }
  glDeleteShader(vs);
  glDeleteShader(fs);

  vs = CompileShader(GL_VERTEX_SHADER, lineVs);
  fs = CompileShader(GL_FRAGMENT_SHADER, lineFs);
  if (vs == 0 || fs == 0 || !LinkShader(lineShader_, vs, fs)) {
    return false;
  }
  glDeleteShader(vs);
  glDeleteShader(fs);

  vs = CompileShader(GL_VERTEX_SHADER, texVs);
  fs = CompileShader(GL_FRAGMENT_SHADER, texFs);
  if (vs == 0 || fs == 0 || !LinkShader(texturedShader_, vs, fs)) {
    return false;
  }
  glDeleteShader(vs);
  glDeleteShader(fs);

  return true;
}

bool Renderer::CreateMeshes() {
  {
    const std::vector<VertexP> cubeVerts = {
        {{-0.5F, -0.5F, -0.5F}}, {{0.5F, -0.5F, -0.5F}}, {{0.5F, 0.5F, -0.5F}}, {{-0.5F, 0.5F, -0.5F}},
        {{-0.5F, -0.5F, 0.5F}},  {{0.5F, -0.5F, 0.5F}},  {{0.5F, 0.5F, 0.5F}},  {{-0.5F, 0.5F, 0.5F}},
    };
    const std::vector<uint32_t> cubeIdx = {
        0, 1, 2, 2, 3, 0,
        4, 5, 6, 6, 7, 4,
        0, 4, 7, 7, 3, 0,
        1, 5, 6, 6, 2, 1,
        3, 2, 6, 6, 7, 3,
        0, 1, 5, 5, 4, 0,
    };
    if (!UploadMesh(cubeVerts, cubeIdx, cubeMesh_)) {
      return false;
    }
  }

  {
    std::vector<VertexP> sphereVerts;
    std::vector<uint32_t> sphereIdx;

    constexpr int segments = 24;
    constexpr int rings = 16;

    for (int r = 0; r <= rings; ++r) {
      const float v = static_cast<float>(r) / static_cast<float>(rings);
      const float phi = v * kPi;
      const float sp = std::sin(phi);
      const float cp = std::cos(phi);
      for (int s = 0; s <= segments; ++s) {
        const float u = static_cast<float>(s) / static_cast<float>(segments);
        const float theta = u * (2.0F * kPi);
        const float ct = std::cos(theta);
        const float st = std::sin(theta);
        sphereVerts.push_back({{sp * ct, sp * st, cp}});
      }
    }

    const int stride = segments + 1;
    for (int r = 0; r < rings; ++r) {
      const int base0 = r * stride;
      const int base1 = (r + 1) * stride;
      for (int s = 0; s < segments; ++s) {
        const uint32_t a = static_cast<uint32_t>(base0 + s);
        const uint32_t b = static_cast<uint32_t>(base1 + s);
        const uint32_t c = static_cast<uint32_t>(base1 + s + 1);
        const uint32_t d = static_cast<uint32_t>(base0 + s + 1);
        sphereIdx.insert(sphereIdx.end(), {a, b, c, a, c, d});
      }
    }

    if (!UploadMesh(sphereVerts, sphereIdx, sphereMesh_)) {
      return false;
    }
  }

  {
    const std::vector<VertexPT> quadVerts = {
        {{0.0F, 0.0F, 0.0F}, {0.0F, 0.0F}},
        {{1.0F, 0.0F, 0.0F}, {1.0F, 0.0F}},
        {{1.0F, 1.0F, 0.0F}, {1.0F, 1.0F}},
        {{0.0F, 1.0F, 0.0F}, {0.0F, 1.0F}},
    };
    const std::vector<uint32_t> quadIdx = {0, 1, 2, 2, 3, 0};
    if (!UploadMesh(quadVerts, quadIdx, quadMesh_, true)) {
      return false;
    }
  }

  return true;
}

bool Renderer::CreateFieldTexture() {
  if (cfg_.fieldImagePath.empty()) {
    return false;
  }

  stbi_set_flip_vertically_on_load(0);
  int width = 0;
  int height = 0;
  int channels = 0;
  unsigned char* pixels = stbi_load(cfg_.fieldImagePath.c_str(), &width, &height, &channels, STBI_rgb_alpha);
  if (pixels == nullptr) {
    return false;
  }

  glGenTextures(1, &fieldTexture_);
  glBindTexture(GL_TEXTURE_2D, fieldTexture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
  glGenerateMipmap(GL_TEXTURE_2D);

  glBindTexture(GL_TEXTURE_2D, 0);
  stbi_image_free(pixels);
  return true;
}

void Renderer::DestroyShader(Shader& shader) const {
  if (shader.id != 0) {
    glDeleteProgram(shader.id);
    shader.id = 0;
  }
}

void Renderer::DestroyMesh(Mesh& mesh) const {
  if (mesh.ebo != 0) {
    glDeleteBuffers(1, &mesh.ebo);
    mesh.ebo = 0;
  }
  if (mesh.vbo != 0) {
    glDeleteBuffers(1, &mesh.vbo);
    mesh.vbo = 0;
  }
  if (mesh.vao != 0) {
    glDeleteVertexArrays(1, &mesh.vao);
    mesh.vao = 0;
  }
  mesh.indexCount = 0;
}

unsigned int Renderer::CompileShader(const unsigned int type, const char* source) const {
  const unsigned int shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);

  int ok = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (ok == GL_TRUE) {
    return shader;
  }

  int len = 0;
  glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);
  std::string log(static_cast<size_t>(std::max(0, len)), '\0');
  if (len > 0) {
    glGetShaderInfoLog(shader, len, nullptr, log.data());
  }

  std::cerr << "Shader compile error: " << log << "\n";
  glDeleteShader(shader);
  return 0;
}

bool Renderer::LinkShader(Shader& shader, const unsigned int vs, const unsigned int fs) const {
  const unsigned int program = glCreateProgram();
  glAttachShader(program, vs);
  glAttachShader(program, fs);
  glLinkProgram(program);

  int ok = 0;
  glGetProgramiv(program, GL_LINK_STATUS, &ok);
  if (ok == GL_TRUE) {
    shader.id = program;
    return true;
  }

  int len = 0;
  glGetProgramiv(program, GL_INFO_LOG_LENGTH, &len);
  std::string log(static_cast<size_t>(std::max(0, len)), '\0');
  if (len > 0) {
    glGetProgramInfoLog(program, len, nullptr, log.data());
  }

  std::cerr << "Shader link error: " << log << "\n";
  glDeleteProgram(program);
  return false;
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

void Renderer::DrawFuel(const glm::mat4& vp, const WorldSnapshot& snap) {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const double nowS = std::chrono::duration_cast<std::chrono::duration<double>>(now).count();

  for (const auto& o : snap.fieldVision) {
    if (NormalizeType(o.type) != "fuel") {
      continue;
    }
    fuelCache_[o.oid] = o;
    fuelLastSeen_[o.oid] = nowS;
  }

  if (fuelCache_.size() > 1500) {
    const double ttl = std::max(1.5, static_cast<double>(cfg_.resourceHardMaxAgeS * 3.0F));
    std::vector<std::string> eraseKeys;
    eraseKeys.reserve(fuelLastSeen_.size());

    for (const auto& [oid, seen] : fuelLastSeen_) {
      if (nowS - seen > ttl) {
        eraseKeys.push_back(oid);
      }
    }
    for (const auto& k : eraseKeys) {
      fuelLastSeen_.erase(k);
      fuelCache_.erase(k);
    }
  }

  if (showAgeFilteredFuel) {
    for (const auto& [oid, o] : fuelCache_) {
      const double age = nowS - fuelLastSeen_[oid];
      if (age > static_cast<double>(cfg_.resourceHardMaxAgeS)) {
        continue;
      }
      const float w = std::exp(-cfg_.collectAgeDecay * static_cast<float>(std::max(0.0, age)));
      const float scale = 0.35F + 0.65F * w;
      const glm::vec4 col = colFuel_ * scale;
      DrawSphere(vp, glm::vec3{static_cast<float>(o.x), static_cast<float>(o.y), fieldZ_ + static_cast<float>(o.z)},
                 fuelRadius_, col);
    }
    return;
  }

  for (const auto& o : snap.fieldVision) {
    if (NormalizeType(o.type) != "fuel") {
      continue;
    }
    DrawSphere(vp, glm::vec3{static_cast<float>(o.x), static_cast<float>(o.y), fieldZ_ + static_cast<float>(o.z)},
               fuelRadius_, colFuel_);
  }
}

void Renderer::DrawTruthFuel(const glm::mat4& vp, const WorldSnapshot& snap) {
  for (const auto& o : snap.truth) {
    DrawSphere(vp, glm::vec3{static_cast<float>(o.x), static_cast<float>(o.y), fieldZ_ + static_cast<float>(o.z)},
               fuelRadius_ * 0.85F, colTruthFuel_);
  }
}

void Renderer::DrawOtherRobots(const glm::mat4& vp, const WorldSnapshot& snap) {
  const float hz = fieldZ_ + obsSide_ * 0.5F;
  for (const auto& o : snap.repulsorVision) {
    DrawBox(vp, glm::vec3{static_cast<float>(o.x), static_cast<float>(o.y), hz},
            glm::vec3{obsSide_, obsSide_, obsSide_}, 0.0F, colOther_);
  }
}

void Renderer::DrawRobotTargets(const glm::mat4& vp, const WorldSnapshot& snap) {
  if (snap.pose.has_value()) {
    const Pose2D& p = snap.pose.value();
    DrawBox(vp,
            glm::vec3{static_cast<float>(p.x), static_cast<float>(p.y), fieldZ_ + robotH_ * 0.5F},
            glm::vec3{robotL_, robotW_, robotH_}, static_cast<float>(glm::degrees(p.thetaRad)), colUs_);

    const float headingLen = std::max(robotL_, robotW_) * 0.95F;
    const glm::vec3 start{static_cast<float>(p.x), static_cast<float>(p.y), fieldZ_ + robotH_ + 0.02F};
    const glm::vec3 end{start.x + headingLen * std::cos(static_cast<float>(p.thetaRad)),
                        start.y + headingLen * std::sin(static_cast<float>(p.thetaRad)),
                        start.z};

    std::vector<VertexPC> lines;
    lines.push_back({start, colHeading_});
    lines.push_back({end, colHeading_});
    DrawLineList(vp, lines, 3.0F);
  }

  if (snap.activeGoal.has_value()) {
    const Pose2D& p = snap.activeGoal.value();
    DrawBox(vp,
            glm::vec3{static_cast<float>(p.x), static_cast<float>(p.y), fieldZ_ + robotH_ * 0.5F},
            glm::vec3{robotL_, robotW_, robotH_}, static_cast<float>(glm::degrees(p.thetaRad)), colActive_);
  }

  if (snap.chosenCollect.has_value()) {
    const Pose2D& p = snap.chosenCollect.value();
    DrawBox(vp,
            glm::vec3{static_cast<float>(p.x), static_cast<float>(p.y), fieldZ_ + robotH_ * 0.5F},
            glm::vec3{robotL_, robotW_, robotH_}, static_cast<float>(glm::degrees(p.thetaRad)), colChosen_);
  }

  if (snap.finalCollect.has_value()) {
    const Pose2D& p = snap.finalCollect.value();
    DrawBox(vp,
            glm::vec3{static_cast<float>(p.x), static_cast<float>(p.y), fieldZ_ + robotH_ * 0.5F},
            glm::vec3{robotL_, robotW_, robotH_}, static_cast<float>(glm::degrees(p.thetaRad)), colFinal_);
  }
}

void Renderer::DrawCameras(const glm::mat4& vp, const WorldSnapshot& snap) {
  if (!snap.pose.has_value() || snap.cameras.empty()) {
    return;
  }

  const Pose2D& rp = snap.pose.value();
  const float rx = static_cast<float>(rp.x);
  const float ry = static_cast<float>(rp.y);
  const float rz = fieldZ_;
  const float yaw = static_cast<float>(rp.thetaRad);

  const float yawCos = std::cos(yaw);
  const float yawSin = std::sin(yaw);

  for (const auto& c : snap.cameras) {
    const float cx = rx + static_cast<float>(c.x) * yawCos - static_cast<float>(c.y) * yawSin;
    const float cy = ry + static_cast<float>(c.x) * yawSin + static_cast<float>(c.y) * yawCos;
    const float cz = rz + static_cast<float>(c.z);

    const float yawWorld = yaw + glm::radians(static_cast<float>(c.yawDeg));
    const float pitchWorld = glm::radians(static_cast<float>(c.pitchDeg));
    const float rollWorld = glm::radians(static_cast<float>(c.rollDeg));

    const float hfov = glm::radians(std::max(1e-6F, static_cast<float>(c.hfovDeg)));
    const float vfov = glm::radians(std::max(1e-6F, static_cast<float>(c.vfovDeg)));
    const float depth = std::max(0.2F, static_cast<float>(c.maxRange));

    DrawBox(vp, glm::vec3{cx, cy, cz}, glm::vec3{0.06F, 0.06F, 0.06F}, glm::degrees(yawWorld), colCam_);

    const float ch = std::cos(yawWorld);
    const float sh = std::sin(yawWorld);
    const float cp = std::cos(pitchWorld);
    const float sp = std::sin(pitchWorld);

    const glm::vec3 forward{cp * ch, cp * sh, sp};
    const glm::vec3 right{-sh, ch, 0.0F};
    const glm::vec3 up{-sp * ch, -sp * sh, cp};

    const float tanH = std::tan(hfov * 0.5F);
    const float tanV = std::tan(vfov * 0.5F);

    const glm::vec3 farCenter = glm::vec3{cx, cy, cz} + forward * depth;
    const glm::vec3 wv = right * (depth * tanH);
    const glm::vec3 hv = up * (depth * tanV);

    const std::array<glm::vec3, 4> corners = {
        farCenter + wv + hv,
        farCenter + wv - hv,
        farCenter - wv - hv,
        farCenter - wv + hv,
    };

    std::vector<VertexPC> lines;
    lines.reserve(256);

    for (const auto& p : corners) {
      lines.push_back({glm::vec3{cx, cy, cz}, colCamFov_});
      lines.push_back({p, colCamFov_});
    }
    for (size_t i = 0; i < corners.size(); ++i) {
      lines.push_back({corners[i], colCamFov_});
      lines.push_back({corners[(i + 1) % corners.size()], colCamFov_});
    }

    const float cr = std::cos(rollWorld);
    const float sr = std::sin(rollWorld);

    const float r00 = ch * cp;
    const float r01 = ch * sp * sr - sh * cr;
    const float r02 = ch * sp * cr + sh * sr;
    const float r10 = sh * cp;
    const float r11 = sh * sp * sr + ch * cr;
    const float r12 = sh * sp * cr - ch * sr;
    const float r20 = -sp;
    const float r21 = cp * sr;
    const float r22 = cp * cr;

    for (const auto& o : snap.fieldVision) {
      const float ox = static_cast<float>(o.x);
      const float oy = static_cast<float>(o.y);
      const float oz = rz + static_cast<float>(o.z);
      const float dx = ox - cx;
      const float dy = oy - cy;
      const float dz = oz - cz;

      const float xCam = r00 * dx + r10 * dy + r20 * dz;
      const float yCam = r01 * dx + r11 * dy + r21 * dz;
      const float zCam = r02 * dx + r12 * dy + r22 * dz;

      glm::vec4 col = colCamRayBad_;
      if (xCam > 1e-6F) {
        const float dyaw = std::atan2(yCam, xCam);
        const float dpitch = std::atan2(zCam, xCam);
        if (std::abs(dyaw) <= hfov * 0.5F && std::abs(dpitch) <= vfov * 0.5F) {
          col = colCamRayOk_;
        }
      }

      lines.push_back({glm::vec3{cx, cy, cz}, col});
      lines.push_back({glm::vec3{ox, oy, oz}, col});
    }

    DrawLineList(vp, lines, 1.0F);
  }
}

void Renderer::DrawBox(
    const glm::mat4& vp,
    const glm::vec3& center,
    const glm::vec3& size,
    const float yawDeg,
    const glm::vec4& color) {
  glm::mat4 model(1.0F);
  model = glm::translate(model, center);
  model = glm::rotate(model, glm::radians(yawDeg), glm::vec3{0.0F, 0.0F, 1.0F});
  model = glm::scale(model, size);

  const glm::mat4 mvp = vp * model;

  glUseProgram(solidShader_.id);
  glUniformMatrix4fv(glGetUniformLocation(solidShader_.id, "uMvp"), 1, GL_FALSE, &mvp[0][0]);
  glUniform4f(glGetUniformLocation(solidShader_.id, "uColor"), color.r, color.g, color.b, color.a);

  glBindVertexArray(cubeMesh_.vao);
  glDrawElements(GL_TRIANGLES, cubeMesh_.indexCount, GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
}

void Renderer::DrawSphere(const glm::mat4& vp, const glm::vec3& center, const float radius, const glm::vec4& color) {
  glm::mat4 model(1.0F);
  model = glm::translate(model, center);
  model = glm::scale(model, glm::vec3{radius, radius, radius});

  const glm::mat4 mvp = vp * model;

  glUseProgram(solidShader_.id);
  glUniformMatrix4fv(glGetUniformLocation(solidShader_.id, "uMvp"), 1, GL_FALSE, &mvp[0][0]);
  glUniform4f(glGetUniformLocation(solidShader_.id, "uColor"), color.r, color.g, color.b, color.a);

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

void Renderer::DrawOverlay(const int width, const int height, const SnapshotBundle& bundle) {
  glDisable(GL_DEPTH_TEST);

  std::vector<std::string> lines;
  lines.push_back(std::string("[C] Camera debug: ") + (showCameraDebug ? "ON" : "OFF"));
  lines.push_back(std::string("[T] Truth fuel: ") + (showTruthFuel ? "ON" : "OFF"));
  lines.push_back(std::string("[A] Age filter: ") + (showAgeFilteredFuel ? "ON" : "OFF"));
  lines.push_back(std::string("Field image: ") + (showFieldImage ? "ON" : "OFF"));
  lines.push_back("Pieces: " + std::to_string(bundle.pieces));
  lines.push_back("Method: " + bundle.method);

  float y = static_cast<float>(height) - 12.0F;
  constexpr float scale = 2.0F;
  for (const auto& line : lines) {
    DrawText2D(10.0F, y, scale, line, glm::vec4{0.92F, 0.92F, 0.92F, 0.9F});
    y -= 8.0F * scale;
  }

  glEnable(GL_DEPTH_TEST);
}

void Renderer::DrawText2D(const float x, const float y, const float scale, const std::string& text, const glm::vec4& color) {
  std::vector<glm::vec3> triangles;
  triangles.reserve(text.size() * 5 * 7 * 6);

  float cx = x;
  for (const char rawCh : ToUpperAscii(text)) {
    const auto glyph = Glyph(rawCh);
    for (int row = 0; row < 7; ++row) {
      for (int col = 0; col < 5; ++col) {
        if (glyph[row][col] != '1') {
          continue;
        }

        const float px = cx + static_cast<float>(col) * scale;
        const float py = y - static_cast<float>(row) * scale;

        triangles.push_back({px, py - scale, 0.0F});
        triangles.push_back({px + scale, py - scale, 0.0F});
        triangles.push_back({px + scale, py, 0.0F});

        triangles.push_back({px, py - scale, 0.0F});
        triangles.push_back({px + scale, py, 0.0F});
        triangles.push_back({px, py, 0.0F});
      }
    }
    cx += 6.0F * scale;
  }

  if (triangles.empty()) {
    return;
  }

  const glm::mat4 ortho = glm::ortho(0.0F, static_cast<float>(width_), 0.0F, static_cast<float>(height_), -1.0F, 1.0F);

  glUseProgram(solidShader_.id);
  glUniformMatrix4fv(glGetUniformLocation(solidShader_.id, "uMvp"), 1, GL_FALSE, &ortho[0][0]);
  glUniform4f(glGetUniformLocation(solidShader_.id, "uColor"), color.r, color.g, color.b, color.a);

  glBindVertexArray(textVao_);
  glBindBuffer(GL_ARRAY_BUFFER, textVbo_);
  glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(triangles.size() * sizeof(glm::vec3)), triangles.data(), GL_DYNAMIC_DRAW);
  glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(triangles.size()));
  glBindVertexArray(0);
}

std::string Renderer::ToUpperAscii(const std::string& s) {
  std::string out = s;
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return out;
}

std::array<std::string, 7> Renderer::Glyph(const char c) {
  static const std::map<char, std::array<std::string, 7>> kGlyphs = {
      {' ', {"00000", "00000", "00000", "00000", "00000", "00000", "00000"}},
      {'[', {"11100", "10000", "10000", "10000", "10000", "10000", "11100"}},
      {']', {"00111", "00001", "00001", "00001", "00001", "00001", "00111"}},
      {':', {"00000", "00100", "00100", "00000", "00100", "00100", "00000"}},
      {'-', {"00000", "00000", "00000", "11111", "00000", "00000", "00000"}},
      {'0', {"01110", "10001", "10011", "10101", "11001", "10001", "01110"}},
      {'1', {"00100", "01100", "00100", "00100", "00100", "00100", "01110"}},
      {'2', {"01110", "10001", "00001", "00010", "00100", "01000", "11111"}},
      {'3', {"11110", "00001", "00001", "01110", "00001", "00001", "11110"}},
      {'4', {"00010", "00110", "01010", "10010", "11111", "00010", "00010"}},
      {'5', {"11111", "10000", "10000", "11110", "00001", "00001", "11110"}},
      {'6', {"01110", "10000", "10000", "11110", "10001", "10001", "01110"}},
      {'7', {"11111", "00001", "00010", "00100", "01000", "01000", "01000"}},
      {'8', {"01110", "10001", "10001", "01110", "10001", "10001", "01110"}},
      {'9', {"01110", "10001", "10001", "01111", "00001", "00001", "01110"}},
      {'A', {"01110", "10001", "10001", "11111", "10001", "10001", "10001"}},
      {'B', {"11110", "10001", "10001", "11110", "10001", "10001", "11110"}},
      {'C', {"01111", "10000", "10000", "10000", "10000", "10000", "01111"}},
      {'D', {"11110", "10001", "10001", "10001", "10001", "10001", "11110"}},
      {'E', {"11111", "10000", "10000", "11110", "10000", "10000", "11111"}},
      {'F', {"11111", "10000", "10000", "11110", "10000", "10000", "10000"}},
      {'G', {"01111", "10000", "10000", "10011", "10001", "10001", "01111"}},
      {'H', {"10001", "10001", "10001", "11111", "10001", "10001", "10001"}},
      {'I', {"11111", "00100", "00100", "00100", "00100", "00100", "11111"}},
      {'J', {"00111", "00010", "00010", "00010", "00010", "10010", "01100"}},
      {'K', {"10001", "10010", "10100", "11000", "10100", "10010", "10001"}},
      {'L', {"10000", "10000", "10000", "10000", "10000", "10000", "11111"}},
      {'M', {"10001", "11011", "10101", "10101", "10001", "10001", "10001"}},
      {'N', {"10001", "11001", "10101", "10011", "10001", "10001", "10001"}},
      {'O', {"01110", "10001", "10001", "10001", "10001", "10001", "01110"}},
      {'P', {"11110", "10001", "10001", "11110", "10000", "10000", "10000"}},
      {'Q', {"01110", "10001", "10001", "10001", "10101", "10010", "01101"}},
      {'R', {"11110", "10001", "10001", "11110", "10100", "10010", "10001"}},
      {'S', {"01111", "10000", "10000", "01110", "00001", "00001", "11110"}},
      {'T', {"11111", "00100", "00100", "00100", "00100", "00100", "00100"}},
      {'U', {"10001", "10001", "10001", "10001", "10001", "10001", "01110"}},
      {'V', {"10001", "10001", "10001", "10001", "10001", "01010", "00100"}},
      {'W', {"10001", "10001", "10001", "10101", "10101", "10101", "01010"}},
      {'X', {"10001", "10001", "01010", "00100", "01010", "10001", "10001"}},
      {'Y', {"10001", "10001", "01010", "00100", "00100", "00100", "00100"}},
      {'Z', {"11111", "00001", "00010", "00100", "01000", "10000", "11111"}},
      {'.', {"00000", "00000", "00000", "00000", "00000", "00100", "00100"}},
      {'/', {"00001", "00010", "00100", "01000", "10000", "00000", "00000"}},
      {'(', {"00010", "00100", "01000", "01000", "01000", "00100", "00010"}},
      {')', {"01000", "00100", "00010", "00010", "00010", "00100", "01000"}},
      {'=', {"00000", "11111", "00000", "11111", "00000", "00000", "00000"}},
      {'+', {"00000", "00100", "00100", "11111", "00100", "00100", "00000"}},
      {'_', {"00000", "00000", "00000", "00000", "00000", "00000", "11111"}},
      {'?', {"01110", "10001", "00001", "00010", "00100", "00000", "00100"}},
      {'!', {"00100", "00100", "00100", "00100", "00100", "00000", "00100"}},
      {',', {"00000", "00000", "00000", "00000", "00110", "00100", "01000"}},
  };

  const auto it = kGlyphs.find(c);
  if (it != kGlyphs.end()) {
    return it->second;
  }

  return {"00000", "00000", "00000", "00000", "00000", "00000", "00000"};
}

}  // namespace repulsor3d
