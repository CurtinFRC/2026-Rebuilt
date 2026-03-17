#include "repulsor3d/render/CadModelRenderFeature.hpp"

#include <GL/glew.h>

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <memory>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include <glm/ext/matrix_transform.hpp>
#include <glm/trigonometric.hpp>

#include "repulsor3d/Renderer.hpp"

namespace repulsor3d {

void CadModelRenderFeature::FlatLitMaterialPass::Apply(const MaterialInstance& material) {
  if (colorUniformLocation_ >= 0) {
    const auto& c = material.definition.baseColor;
    glUniform4f(colorUniformLocation_, c.r, c.g, c.b, c.a);
  }
}

void CadModelRenderFeature::WireframeMaterialPass::Apply(const MaterialInstance& material) {
  backend_.SetWireframeMode(material.definition.wireframe);
}

CadModelRenderFeature::~CadModelRenderFeature() {
  for (auto& [_, mesh] : meshCache_) {
    DestroyMesh(mesh);
  }
  meshCache_.clear();

  shader_.Reset();
}

bool CadModelRenderFeature::Initialize(Renderer& renderer) {
  renderer_ = &renderer;
  geometryProvider_ = &renderer.GetGeometryProvider();
  backend_ = &renderer.GetRenderBackend();

  if (initialized_) {
    return true;
  }

  initialized_ = CreateShader();
  return initialized_;
}

void CadModelRenderFeature::Render(const RenderFeatureContext& context, const RendererDrawApi& /*drawApi*/) {
  if (!initialized_ || shader_.Get() == 0 || geometryProvider_ == nullptr || backend_ == nullptr) {
    return;
  }

  backend_->UseProgram(shader_.Get());
  glUniform3f(uLightDirLoc_, -0.3F, -0.5F, -1.0F);

  for (const auto& command : context.commandBuffer) {
    std::visit(
        [&](const auto& typed) {
          using T = std::decay_t<decltype(typed)>;
          if constexpr (std::is_same_v<T, DrawMeshInstanceCommand>) {
            const auto& instance = typed.primitive;
            const GpuMesh* mesh = GetOrLoadMesh(instance.assetPath);
            if (mesh == nullptr || mesh->vertexCount <= 0) {
              return;
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

            MaterialInstance material;
            material.definition.baseColor = instance.color;
            material.definition.wireframe = instance.wireframe;
            materialPipeline_.Apply(material);

            backend_->BindVertexArray(mesh->vao.Get());
            backend_->DrawTriangles(mesh->vertexCount);
            backend_->BindVertexArray(0);
          }
        },
        command);
  }

  backend_->SetWireframeMode(false);
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

  uMvpLoc_ = glGetUniformLocation(shader_.Get(), "uMvp");
  uModelLoc_ = glGetUniformLocation(shader_.Get(), "uModel");
  uColorLoc_ = glGetUniformLocation(shader_.Get(), "uColor");
  uLightDirLoc_ = glGetUniformLocation(shader_.Get(), "uLightDir");
  materialPipeline_.AddPass(std::make_unique<FlatLitMaterialPass>(uColorLoc_));
  if (backend_ != nullptr) {
    materialPipeline_.AddPass(std::make_unique<WireframeMaterialPass>(*backend_));
  }
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

bool CadModelRenderFeature::LinkShader(GlProgramHandle& program, const unsigned int vs, const unsigned int fs) {
  const unsigned int p = glCreateProgram();
  glAttachShader(p, vs);
  glAttachShader(p, fs);
  glLinkProgram(p);

  int ok = 0;
  glGetProgramiv(p, GL_LINK_STATUS, &ok);
  if (ok == GL_TRUE) {
    program.Set(p);
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

bool CadModelRenderFeature::UploadMesh(const PositionNormalMesh& cpu, GpuMesh& gpu) {
  if (cpu.vertices.empty()) {
    return false;
  }

  std::vector<VertexPN> packed;
  packed.reserve(cpu.vertices.size());
  for (const auto& v : cpu.vertices) {
    packed.push_back({v.position, v.normal});
  }

  unsigned int vaoId = 0;
  unsigned int vboId = 0;
  glGenVertexArrays(1, &vaoId);
  glGenBuffers(1, &vboId);
  if (vaoId == 0 || vboId == 0) {
    DestroyMesh(gpu);
    return false;
  }

  gpu.vao.Set(vaoId);
  gpu.vbo.Set(vboId);

  glBindVertexArray(gpu.vao.Get());
  glBindBuffer(GL_ARRAY_BUFFER, gpu.vbo.Get());
  glBufferData(
      GL_ARRAY_BUFFER,
      static_cast<GLsizeiptr>(packed.size() * sizeof(VertexPN)),
      packed.data(),
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

  gpu.vertexCount = static_cast<int>(packed.size());
  return true;
}

void CadModelRenderFeature::DestroyMesh(GpuMesh& gpu) {
  gpu.vbo.Reset();
  gpu.vao.Reset();
  gpu.vertexCount = 0;
}

const CadModelRenderFeature::GpuMesh* CadModelRenderFeature::GetOrLoadMesh(const std::string& assetPath) {
  const auto cached = meshCache_.find(assetPath);
  if (cached != meshCache_.end()) {
    return &cached->second;
  }

  if (geometryProvider_ == nullptr) {
    return nullptr;
  }

  PositionNormalMesh cpu;
  if (!geometryProvider_->GetCadMesh(assetPath, cpu)) {
    return nullptr;
  }

  GpuMesh gpu;
  if (!UploadMesh(cpu, gpu)) {
    return nullptr;
  }

  auto [it, inserted] = meshCache_.emplace(assetPath, std::move(gpu));
  if (!inserted) {
    DestroyMesh(gpu);
  }
  return &it->second;
}

}  // namespace repulsor3d
