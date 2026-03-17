#include "repulsor3d/render/CadModelRenderFeature.hpp"

#include <GL/glew.h>

#include <array>
#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <glm/common.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/geometric.hpp>
#include <glm/trigonometric.hpp>

namespace repulsor3d {
namespace {

constexpr float kEpsilon = 1e-6F;

}  // namespace

CadModelRenderFeature::~CadModelRenderFeature() {
  for (auto& [_, mesh] : meshCache_) {
    DestroyMesh(mesh);
  }
  meshCache_.clear();

  if (shader_ != 0) {
    glDeleteProgram(shader_);
    shader_ = 0;
  }
}

bool CadModelRenderFeature::Initialize(Renderer& /*renderer*/) {
  if (initialized_) {
    return true;
  }

  initialized_ = CreateShader();
  return initialized_;
}

void CadModelRenderFeature::Render(const RenderFeatureContext& context, const RendererDrawApi& /*drawApi*/) {
  if (!initialized_ || shader_ == 0 || context.frame.meshInstances.empty()) {
    return;
  }

  glUseProgram(shader_);
  glUniform3f(uLightDirLoc_, -0.3F, -0.5F, -1.0F);

  bool anyWireframe = false;
  for (const auto& instance : context.frame.meshInstances) {
    const GpuMesh* mesh = GetOrLoadMesh(instance.assetPath);
    if (mesh == nullptr || mesh->vertexCount <= 0) {
      continue;
    }

    glm::mat4 model(1.0F);
    model = glm::translate(model, instance.position);
    model = glm::rotate(model, glm::radians(instance.rotationDeg.x), glm::vec3{1.0F, 0.0F, 0.0F});
    model = glm::rotate(model, glm::radians(instance.rotationDeg.y), glm::vec3{0.0F, 1.0F, 0.0F});
    model = glm::rotate(model, glm::radians(instance.rotationDeg.z), glm::vec3{0.0F, 0.0F, 1.0F});
    model = glm::scale(model, instance.scale);

    const glm::mat4 mvp = context.viewProjection * model;
    glUniformMatrix4fv(uMvpLoc_, 1, GL_FALSE, &mvp[0][0]);
    glUniformMatrix4fv(uModelLoc_, 1, GL_FALSE, &model[0][0]);
    glUniform4f(uColorLoc_, instance.color.r, instance.color.g, instance.color.b, instance.color.a);

    if (instance.wireframe) {
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
      anyWireframe = true;
    } else {
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    glBindVertexArray(mesh->vao);
    glDrawArrays(GL_TRIANGLES, 0, mesh->vertexCount);
    glBindVertexArray(0);
  }

  if (anyWireframe) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }
}

bool CadModelRenderFeature::CreateShader() {
  constexpr const char* vsSrc = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 uMvp;
uniform mat4 uModel;
out vec3 vNormal;
void main() {
  vNormal = mat3(transpose(inverse(uModel))) * aNormal;
  gl_Position = uMvp * vec4(aPos, 1.0);
}
)";

  constexpr const char* fsSrc = R"(
#version 330 core
in vec3 vNormal;
uniform vec4 uColor;
uniform vec3 uLightDir;
out vec4 FragColor;
void main() {
  vec3 n = normalize(vNormal);
  float diff = max(dot(n, normalize(-uLightDir)), 0.0);
  float lighting = 0.25 + 0.75 * diff;
  FragColor = vec4(uColor.rgb * lighting, uColor.a);
}
)";

  const unsigned int vs = CompileShader(GL_VERTEX_SHADER, vsSrc);
  const unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fsSrc);
  if (vs == 0 || fs == 0) {
    if (vs != 0) {
      glDeleteShader(vs);
    }
    if (fs != 0) {
      glDeleteShader(fs);
    }
    return false;
  }

  if (!LinkShader(shader_, vs, fs)) {
    glDeleteShader(vs);
    glDeleteShader(fs);
    return false;
  }

  glDeleteShader(vs);
  glDeleteShader(fs);

  uMvpLoc_ = glGetUniformLocation(shader_, "uMvp");
  uModelLoc_ = glGetUniformLocation(shader_, "uModel");
  uColorLoc_ = glGetUniformLocation(shader_, "uColor");
  uLightDirLoc_ = glGetUniformLocation(shader_, "uLightDir");
  return uMvpLoc_ >= 0 && uModelLoc_ >= 0 && uColorLoc_ >= 0 && uLightDirLoc_ >= 0;
}

unsigned int CadModelRenderFeature::CompileShader(const unsigned int type, const char* source) {
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
  std::cerr << "CAD shader compile error: " << log << "\n";

  glDeleteShader(shader);
  return 0;
}

bool CadModelRenderFeature::LinkShader(unsigned int& program, const unsigned int vs, const unsigned int fs) {
  const unsigned int p = glCreateProgram();
  glAttachShader(p, vs);
  glAttachShader(p, fs);
  glLinkProgram(p);

  int ok = 0;
  glGetProgramiv(p, GL_LINK_STATUS, &ok);
  if (ok == GL_TRUE) {
    program = p;
    return true;
  }

  int len = 0;
  glGetProgramiv(p, GL_INFO_LOG_LENGTH, &len);
  std::string log(static_cast<size_t>(std::max(0, len)), '\0');
  if (len > 0) {
    glGetProgramInfoLog(p, len, nullptr, log.data());
  }
  std::cerr << "CAD shader link error: " << log << "\n";

  glDeleteProgram(p);
  return false;
}

glm::vec3 CadModelRenderFeature::NormalizeSafe(const glm::vec3& v) {
  const float len = glm::length(v);
  if (len <= kEpsilon) {
    return glm::vec3{0.0F, 0.0F, 1.0F};
  }
  return v / len;
}

glm::vec3 CadModelRenderFeature::ComputeFallbackNormal(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
  const glm::vec3 n = glm::cross(b - a, c - a);
  return NormalizeSafe(n);
}

bool CadModelRenderFeature::UploadMesh(const CpuMesh& cpu, GpuMesh& gpu) {
  if (cpu.vertices.empty()) {
    return false;
  }

  glGenVertexArrays(1, &gpu.vao);
  glGenBuffers(1, &gpu.vbo);
  if (gpu.vao == 0 || gpu.vbo == 0) {
    DestroyMesh(gpu);
    return false;
  }

  glBindVertexArray(gpu.vao);
  glBindBuffer(GL_ARRAY_BUFFER, gpu.vbo);
  glBufferData(
      GL_ARRAY_BUFFER,
      static_cast<GLsizeiptr>(cpu.vertices.size() * sizeof(VertexPN)),
      cpu.vertices.data(),
      GL_STATIC_DRAW);

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), reinterpret_cast<void*>(offsetof(VertexPN, pos)));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(
      1,
      3,
      GL_FLOAT,
      GL_FALSE,
      sizeof(VertexPN),
      reinterpret_cast<void*>(offsetof(VertexPN, normal)));
  glBindVertexArray(0);

  gpu.vertexCount = static_cast<int>(cpu.vertices.size());
  return true;
}

void CadModelRenderFeature::DestroyMesh(GpuMesh& gpu) {
  if (gpu.vbo != 0) {
    glDeleteBuffers(1, &gpu.vbo);
    gpu.vbo = 0;
  }
  if (gpu.vao != 0) {
    glDeleteVertexArrays(1, &gpu.vao);
    gpu.vao = 0;
  }
  gpu.vertexCount = 0;
}

bool CadModelRenderFeature::ParseBinaryStl(const std::string& filePath, CpuMesh& outMesh) {
  std::ifstream in(filePath, std::ios::binary);
  if (!in) {
    return false;
  }

  in.seekg(0, std::ios::end);
  const std::streamoff fileSize = in.tellg();
  if (fileSize < 84) {
    return false;
  }
  in.seekg(0, std::ios::beg);

  std::array<char, 80> header{};
  in.read(header.data(), static_cast<std::streamsize>(header.size()));

  uint32_t triCount = 0;
  in.read(reinterpret_cast<char*>(&triCount), sizeof(uint32_t));
  if (!in) {
    return false;
  }

  const std::uint64_t expected = 84ULL + static_cast<std::uint64_t>(triCount) * 50ULL;
  if (expected != static_cast<std::uint64_t>(fileSize)) {
    return false;
  }

  outMesh.vertices.clear();
  outMesh.vertices.reserve(static_cast<size_t>(triCount) * 3);

  for (uint32_t i = 0; i < triCount; ++i) {
    float nx = 0.0F;
    float ny = 0.0F;
    float nz = 1.0F;
    glm::vec3 p[3];
    in.read(reinterpret_cast<char*>(&nx), sizeof(float));
    in.read(reinterpret_cast<char*>(&ny), sizeof(float));
    in.read(reinterpret_cast<char*>(&nz), sizeof(float));

    for (int j = 0; j < 3; ++j) {
      in.read(reinterpret_cast<char*>(&p[j].x), sizeof(float));
      in.read(reinterpret_cast<char*>(&p[j].y), sizeof(float));
      in.read(reinterpret_cast<char*>(&p[j].z), sizeof(float));
    }

    std::uint16_t attr = 0;
    in.read(reinterpret_cast<char*>(&attr), sizeof(std::uint16_t));
    (void)attr;

    if (!in) {
      return false;
    }

    const glm::vec3 rawNormal{nx, ny, nz};
    glm::vec3 normal = NormalizeSafe(rawNormal);
    if (glm::length(rawNormal) <= kEpsilon) {
      normal = ComputeFallbackNormal(p[0], p[1], p[2]);
    }

    outMesh.vertices.push_back({p[0], normal});
    outMesh.vertices.push_back({p[1], normal});
    outMesh.vertices.push_back({p[2], normal});
  }

  return !outMesh.vertices.empty();
}

bool CadModelRenderFeature::ParseAsciiStl(const std::string& filePath, CpuMesh& outMesh) {
  std::ifstream in(filePath);
  if (!in) {
    return false;
  }

  outMesh.vertices.clear();

  std::string token;
  while (in >> token) {
    if (token != "facet") {
      continue;
    }

    if (!(in >> token) || token != "normal") {
      return false;
    }

    float nx = 0.0F;
    float ny = 0.0F;
    float nz = 1.0F;
    if (!(in >> nx >> ny >> nz)) {
      return false;
    }

    if (!(in >> token) || token != "outer") {
      return false;
    }
    if (!(in >> token) || token != "loop") {
      return false;
    }

    glm::vec3 p[3];
    for (auto& v : p) {
      if (!(in >> token) || token != "vertex") {
        return false;
      }
      if (!(in >> v.x >> v.y >> v.z)) {
        return false;
      }
    }

    if (!(in >> token) || token != "endloop") {
      return false;
    }
    if (!(in >> token) || token != "endfacet") {
      return false;
    }

    const glm::vec3 rawNormal{nx, ny, nz};
    glm::vec3 normal = NormalizeSafe(rawNormal);
    if (glm::length(rawNormal) <= kEpsilon) {
      normal = ComputeFallbackNormal(p[0], p[1], p[2]);
    }

    outMesh.vertices.push_back({p[0], normal});
    outMesh.vertices.push_back({p[1], normal});
    outMesh.vertices.push_back({p[2], normal});
  }

  return !outMesh.vertices.empty();
}

bool CadModelRenderFeature::LoadStl(const std::string& filePath, CpuMesh& outMesh) {
  if (ParseBinaryStl(filePath, outMesh)) {
    return true;
  }
  return ParseAsciiStl(filePath, outMesh);
}

std::string CadModelRenderFeature::ResolveAssetPath(const std::string& assetPath) {
  namespace fs = std::filesystem;
  if (assetPath.empty()) {
    return "";
  }

  const fs::path raw(assetPath);
  if (raw.is_absolute() && fs::exists(raw) && fs::is_regular_file(raw)) {
    return fs::absolute(raw).string();
  }

  const fs::path cwd = fs::current_path();
  const std::vector<fs::path> candidates = {
      raw,
      cwd / raw,
      cwd / "assets" / raw,
      cwd / "repulsor_3d_sim_cpp" / raw,
      cwd / "repulsor_3d_sim_cpp" / "assets" / raw,
      cwd / ".." / raw,
      cwd / ".." / "repulsor_3d_sim_cpp" / raw,
      cwd / ".." / "repulsor_3d_sim_cpp" / "assets" / raw,
  };

  for (const auto& candidate : candidates) {
    if (fs::exists(candidate) && fs::is_regular_file(candidate)) {
      return fs::absolute(candidate).string();
    }
  }

  return "";
}

const CadModelRenderFeature::GpuMesh* CadModelRenderFeature::GetOrLoadMesh(const std::string& assetPath) {
  const std::string resolved = ResolveAssetPath(assetPath);
  if (resolved.empty()) {
    return nullptr;
  }

  const auto cached = meshCache_.find(resolved);
  if (cached != meshCache_.end()) {
    return &cached->second;
  }

  CpuMesh cpu;
  if (!LoadStl(resolved, cpu)) {
    std::cerr << "CAD feature: failed to load STL '" << resolved << "'\n";
    return nullptr;
  }

  GpuMesh gpu;
  if (!UploadMesh(cpu, gpu)) {
    std::cerr << "CAD feature: failed to upload mesh '" << resolved << "'\n";
    return nullptr;
  }

  auto [it, inserted] = meshCache_.emplace(resolved, gpu);
  if (!inserted) {
    DestroyMesh(gpu);
    return &it->second;
  }
  return &it->second;
}

}  // namespace repulsor3d
