#include "repulsor3d/render/CadModelRenderFeature.hpp"

#include <GL/glew.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include <glm/ext/matrix_transform.hpp>
#include <glm/common.hpp>
#include <glm/trigonometric.hpp>

#include "repulsor3d/Renderer.hpp"

namespace repulsor3d {

CadModelRenderFeature::CadModelRenderFeature(
    const RenderPass renderPass,
    std::string featureName,
    std::vector<std::string> dependencies)
    : renderPass_(renderPass), featureName_(std::move(featureName)), dependencies_(std::move(dependencies)) {}

void CadModelRenderFeature::FlatLitMaterialPass::Apply(const MaterialInstance& material) {
  if (colorUniformLocation_ >= 0) {
    const auto& c = material.definition.baseColor;
    backend_.SetUniformVec4(colorUniformLocation_, c.r, c.g, c.b, c.a);
  }
}

void CadModelRenderFeature::WireframeMaterialPass::Apply(const MaterialInstance& material) {
  backend_.SetWireframeMode(material.definition.wireframe);
}

CadModelRenderFeature::~CadModelRenderFeature() {
  for (auto& [_, pending] : pendingLoads_) {
    if (pending.future.valid()) {
      try {
        pending.future.wait();
      } catch (...) {
      }
    }
  }
  pendingLoads_.clear();

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
  backend_->SetUniformVec3(uLightDirLoc_, -0.3F, -0.5F, -1.0F);

  for (const auto& command : context.commandBuffer) {
    std::visit(
        [&](const auto& typed) {
          using T = std::decay_t<decltype(typed)>;
          if constexpr (std::is_same_v<T, DrawMeshInstanceCommand>) {
            if (typed.pass != renderPass_) {
              return;
            }
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
            if (instance.centerOnMeshBounds) {
              model = model * glm::translate(glm::mat4(1.0F), glm::vec3{-mesh->boundsCenter.x, -mesh->boundsCenter.y, 0.0F});
            }

            const glm::mat4 mvp = context.viewProjection * model;
            backend_->SetUniformMat4(uMvpLoc_, &mvp[0][0]);
            backend_->SetUniformMat4(uModelLoc_, &model[0][0]);
            backend_->SetUniform1f(uUseAssetColorLoc_, instance.useAssetColor ? 1.0F : 0.0F);

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
layout(location = 2) in vec4 aColor;
uniform mat4 uMvp;
uniform mat4 uModel;
out vec3 vNormal;
out vec4 vColor;
void main() {
  vNormal = mat3(transpose(inverse(uModel))) * aNormal;
  vColor = aColor;
  gl_Position = uMvp * vec4(aPos, 1.0);
}
)";

  constexpr const char* fsSrc = R"(
#version 330 core
in vec3 vNormal;
in vec4 vColor;
uniform vec4 uColor;
uniform vec3 uLightDir;
uniform float uUseAssetColor;
out vec4 FragColor;
void main() {
  vec3 n = normalize(vNormal);
  float diff = max(dot(n, normalize(-uLightDir)), 0.0);
  float lighting = 0.25 + 0.75 * diff;
  float useAssetColor = clamp(uUseAssetColor, 0.0, 1.0);
  vec3 baseColor = mix(uColor.rgb, vColor.rgb * uColor.rgb, useAssetColor);
  float alpha = uColor.a * mix(1.0, vColor.a, useAssetColor);
  FragColor = vec4(baseColor * lighting, alpha);
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

  uMvpLoc_ = backend_->GetUniformLocation(shader_.Get(), "uMvp");
  uModelLoc_ = backend_->GetUniformLocation(shader_.Get(), "uModel");
  uColorLoc_ = backend_->GetUniformLocation(shader_.Get(), "uColor");
  uLightDirLoc_ = backend_->GetUniformLocation(shader_.Get(), "uLightDir");
  uUseAssetColorLoc_ = backend_->GetUniformLocation(shader_.Get(), "uUseAssetColor");
  materialPipeline_.AddPass(std::make_unique<FlatLitMaterialPass>(*backend_, uColorLoc_));
  if (backend_ != nullptr) {
    materialPipeline_.AddPass(std::make_unique<WireframeMaterialPass>(*backend_));
  }
  return uMvpLoc_ >= 0 && uModelLoc_ >= 0 && uColorLoc_ >= 0 && uLightDirLoc_ >= 0 && uUseAssetColorLoc_ >= 0;
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

bool CadModelRenderFeature::UploadMesh(const PositionNormalMesh& cpu, GpuMesh& gpu, IRenderBackend& backend) {
  if (cpu.vertices.empty()) {
    return false;
  }

  using Vertex = PositionNormalMesh::Vertex;
  const auto& vertices = cpu.vertices;

  const unsigned int vaoId = backend.CreateVertexArray();
  const unsigned int vboId = backend.CreateBuffer();
  if (vaoId == 0 || vboId == 0) {
    DestroyMesh(gpu);
    return false;
  }

  gpu.vao.Set(vaoId);
  gpu.vbo.Set(vboId);

  backend.BindVertexArray(gpu.vao.Get());
  backend.BindArrayBuffer(gpu.vbo.Get());
  backend.UploadArrayBufferData(vertices.size() * sizeof(Vertex), vertices.data(), false);

  backend.EnableVertexAttrib(0);
  backend.DefineVertexAttribFloat(0, 3, sizeof(Vertex), offsetof(Vertex, position));
  backend.EnableVertexAttrib(1);
  backend.DefineVertexAttribFloat(1, 3, sizeof(Vertex), offsetof(Vertex, normal));
  backend.EnableVertexAttrib(2);
  backend.DefineVertexAttribFloat(2, 4, sizeof(Vertex), offsetof(Vertex, color));
  backend.BindVertexArray(0);

  glm::vec3 minP = vertices.front().position;
  glm::vec3 maxP = vertices.front().position;
  for (const auto& v : vertices) {
    minP = glm::min(minP, v.position);
    maxP = glm::max(maxP, v.position);
  }
  gpu.boundsCenter = (minP + maxP) * 0.5F;
  gpu.vertexCount = static_cast<int>(vertices.size());
  return true;
}

void CadModelRenderFeature::DestroyMesh(GpuMesh& gpu) {
  gpu.vbo.Reset();
  gpu.vao.Reset();
  gpu.vertexCount = 0;
  gpu.boundsCenter = glm::vec3{0.0F, 0.0F, 0.0F};
}

const CadModelRenderFeature::GpuMesh* CadModelRenderFeature::GetOrLoadMesh(const std::string& assetPath) {
  const auto cached = meshCache_.find(assetPath);
  if (cached != meshCache_.end()) {
    return &cached->second;
  }
  if (failedLoads_.find(assetPath) != failedLoads_.end()) {
    return nullptr;
  }

  if (geometryProvider_ == nullptr) {
    return nullptr;
  }

  const auto pendingIt = pendingLoads_.find(assetPath);
  if (pendingIt != pendingLoads_.end()) {
    if (pendingIt->second.future.valid() &&
        pendingIt->second.future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      PositionNormalMesh cpu = pendingIt->second.future.get();
      pendingLoads_.erase(pendingIt);
      if (cpu.vertices.empty()) {
        failedLoads_.insert(assetPath);
        std::cerr << "CAD load failed: " << assetPath << "\n";
        return nullptr;
      }

      GpuMesh gpu;
      if (backend_ == nullptr || !UploadMesh(cpu, gpu, *backend_)) {
        failedLoads_.insert(assetPath);
        std::cerr << "CAD upload failed: " << assetPath << " (vertices=" << cpu.vertices.size() << ")\n";
        return nullptr;
      }

      auto [it, inserted] = meshCache_.emplace(assetPath, std::move(gpu));
      if (!inserted) {
        DestroyMesh(gpu);
      } else {
        std::cerr << "CAD loaded: " << assetPath << " (vertices=" << cpu.vertices.size() << ")\n";
      }
      return &it->second;
    }
    return nullptr;
  }

  IGeometryProvider* provider = geometryProvider_;
  pendingLoads_.emplace(
      assetPath,
      PendingLoad{
          std::async(std::launch::async, [provider, assetPath]() {
            PositionNormalMesh cpu;
            if (provider == nullptr || !provider->GetCadMesh(assetPath, cpu)) {
              return PositionNormalMesh{};
            }
            return cpu;
          })});
  std::cerr << "CAD loading started: " << assetPath << "\n";
  return nullptr;
}

}  // namespace repulsor3d
