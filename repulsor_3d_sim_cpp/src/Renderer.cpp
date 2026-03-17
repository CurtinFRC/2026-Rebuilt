#include "repulsor3d/Renderer.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <map>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>

#include "repulsor3d/render/RepulsorSeasonModelBuilder.hpp"

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
  glBufferData(
      GL_ARRAY_BUFFER,
      static_cast<GLsizeiptr>(vertices.size() * sizeof(TVertex)),
      vertices.data(),
      GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ebo);
  glBufferData(
      GL_ELEMENT_ARRAY_BUFFER,
      static_cast<GLsizeiptr>(indices.size() * sizeof(uint32_t)),
      indices.data(),
      GL_STATIC_DRAW);

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(TVertex), reinterpret_cast<void*>(0));

  if (withUv) {
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(
        1,
        2,
        GL_FLOAT,
        GL_FALSE,
        sizeof(TVertex),
        reinterpret_cast<void*>(offsetof(Renderer::VertexPT, uv)));
  }

  glBindVertexArray(0);
  mesh.indexCount = static_cast<int>(indices.size());
  return true;
}

}  // namespace

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
    sceneBuilder_ = std::make_unique<RepulsorSeasonModelBuilder>(cfg_);
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

  namespace fs = std::filesystem;
  std::vector<fs::path> candidates;
  candidates.emplace_back(cfg_.fieldImagePath);

  const fs::path cwd = fs::current_path();
  candidates.emplace_back(cwd / cfg_.fieldImagePath);
  candidates.emplace_back(cwd / "repulsor_3d_sim" / cfg_.fieldImagePath);
  candidates.emplace_back(cwd / "repulsor_3d_sim_cpp" / cfg_.fieldImagePath);

  fs::path resolved;
  for (const auto& candidate : candidates) {
    if (fs::exists(candidate) && fs::is_regular_file(candidate)) {
      resolved = candidate;
      break;
    }
  }

  if (resolved.empty()) {
    return false;
  }

  stbi_set_flip_vertically_on_load(0);
  int width = 0;
  int height = 0;
  int channels = 0;
  const std::string texturePath = resolved.string();
  unsigned char* pixels = stbi_load(texturePath.c_str(), &width, &height, &channels, STBI_rgb_alpha);
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

void Renderer::DrawBox(const glm::mat4& vp, const BoxPrimitive& primitive) {
  glm::mat4 model(1.0F);
  model = glm::translate(model, primitive.center);
  model = glm::rotate(model, glm::radians(primitive.yawDeg), glm::vec3{0.0F, 0.0F, 1.0F});
  model = glm::scale(model, primitive.size);

  const glm::mat4 mvp = vp * model;

  glUseProgram(solidShader_.id);
  glUniformMatrix4fv(glGetUniformLocation(solidShader_.id, "uMvp"), 1, GL_FALSE, &mvp[0][0]);
  glUniform4f(
      glGetUniformLocation(solidShader_.id, "uColor"),
      primitive.color.r,
      primitive.color.g,
      primitive.color.b,
      primitive.color.a);

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
  glUniform4f(
      glGetUniformLocation(solidShader_.id, "uColor"),
      primitive.color.r,
      primitive.color.g,
      primitive.color.b,
      primitive.color.a);

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

void Renderer::DrawOverlay(const int width, const int height, const std::vector<OverlayLine>& lines) {
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
