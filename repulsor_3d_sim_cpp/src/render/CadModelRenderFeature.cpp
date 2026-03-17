#include "repulsor3d/render/CadModelRenderFeature.hpp"

#include <GL/glew.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <variant>
#include <vector>

#include <glm/ext/matrix_transform.hpp>
#include <glm/common.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/trigonometric.hpp>

#include "repulsor3d/Renderer.hpp"

namespace repulsor3d {
namespace {

std::uint32_t FloatBits(const float value) {
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

struct VertexKey {
  std::uint32_t px = 0;
  std::uint32_t py = 0;
  std::uint32_t pz = 0;
  std::uint32_t nx = 0;
  std::uint32_t ny = 0;
  std::uint32_t nz = 0;
  std::uint32_t cr = 0;
  std::uint32_t cg = 0;
  std::uint32_t cb = 0;
  std::uint32_t ca = 0;

  bool operator==(const VertexKey& other) const {
    return px == other.px && py == other.py && pz == other.pz &&
           nx == other.nx && ny == other.ny && nz == other.nz &&
           cr == other.cr && cg == other.cg && cb == other.cb && ca == other.ca;
  }
};

struct VertexKeyHash {
  std::size_t operator()(const VertexKey& key) const {
    std::size_t h = 1469598103934665603ULL;
    auto mix = [&](const std::uint32_t v) {
      h ^= static_cast<std::size_t>(v);
      h *= 1099511628211ULL;
    };
    mix(key.px);
    mix(key.py);
    mix(key.pz);
    mix(key.nx);
    mix(key.ny);
    mix(key.nz);
    mix(key.cr);
    mix(key.cg);
    mix(key.cb);
    mix(key.ca);
    return h;
  }
};

VertexKey ToVertexKey(const PositionNormalMesh::Vertex& v) {
  return VertexKey{
      .px = FloatBits(v.position.x),
      .py = FloatBits(v.position.y),
      .pz = FloatBits(v.position.z),
      .nx = FloatBits(v.normal.x),
      .ny = FloatBits(v.normal.y),
      .nz = FloatBits(v.normal.z),
      .cr = FloatBits(v.color.r),
      .cg = FloatBits(v.color.g),
      .cb = FloatBits(v.color.b),
      .ca = FloatBits(v.color.a)};
}

glm::vec3 ComputeTrimmedBoundsCenterXY(const std::vector<PositionNormalMesh::Vertex>& vertices) {
  if (vertices.empty()) {
    return glm::vec3{0.0F, 0.0F, 0.0F};
  }

  std::size_t sampleCount = vertices.size();
  std::size_t stride = 1;
  constexpr std::size_t kMaxSamples = 500000;
  if (sampleCount > kMaxSamples) {
    stride = (sampleCount + kMaxSamples - 1) / kMaxSamples;
    sampleCount = (vertices.size() + stride - 1) / stride;
  }

  std::vector<float> xs;
  std::vector<float> ys;
  xs.reserve(sampleCount);
  ys.reserve(sampleCount);
  float minZ = std::numeric_limits<float>::max();
  float maxZ = std::numeric_limits<float>::lowest();
  for (std::size_t i = 0; i < vertices.size(); i += stride) {
    const auto& p = vertices[i].position;
    xs.push_back(p.x);
    ys.push_back(p.y);
    minZ = std::min(minZ, p.z);
    maxZ = std::max(maxZ, p.z);
  }
  if (xs.empty() || ys.empty()) {
    return glm::vec3{0.0F, 0.0F, 0.0F};
  }

  std::sort(xs.begin(), xs.end());
  std::sort(ys.begin(), ys.end());
  auto quantile = [](const std::vector<float>& data, const float q) {
    const std::size_t idx = static_cast<std::size_t>(q * static_cast<float>(data.size() - 1));
    return data[idx];
  };

  const float xLo = quantile(xs, 0.01F);
  const float xHi = quantile(xs, 0.99F);
  const float yLo = quantile(ys, 0.01F);
  const float yHi = quantile(ys, 0.99F);
  return glm::vec3{(xLo + xHi) * 0.5F, (yLo + yHi) * 0.5F, (minZ + maxZ) * 0.5F};
}

}  // namespace

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

  if (renderPass_ == RenderPass::Opaque) {
    backend_->SetBlendEnabled(false);
  } else {
    backend_->SetBlendEnabled(true);
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
            if (mesh == nullptr || (mesh->vertexCount <= 0 && mesh->indexCount <= 0)) {
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
            const glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(model)));
            backend_->SetUniformMat4(uMvpLoc_, &mvp[0][0]);
            backend_->SetUniformMat3(uNormalMatrixLoc_, &normalMatrix[0][0]);
            backend_->SetUniform1f(uUseAssetColorLoc_, instance.useAssetColor ? 1.0F : 0.0F);

            MaterialInstance material;
            material.definition.baseColor = instance.color;
            material.definition.wireframe = instance.wireframe;
            materialPipeline_.Apply(material);

            backend_->BindVertexArray(mesh->vao.Get());
            if (mesh->indexCount > 0) {
              backend_->DrawIndexedTriangles(mesh->indexCount);
            } else {
              backend_->DrawTriangles(mesh->vertexCount);
            }
            backend_->BindVertexArray(0);
          }
        },
        command);
  }

  backend_->SetBlendEnabled(true);
  backend_->SetWireframeMode(false);
}

bool CadModelRenderFeature::CreateShader() {
  constexpr const char* vsSrc = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aColor;
uniform mat4 uMvp;
uniform mat3 uNormalMatrix;
uniform vec3 uLightDir;
out vec4 vColor;
out float vLighting;
void main() {
  vec3 n = normalize(uNormalMatrix * aNormal);
  float diff = max(dot(n, normalize(-uLightDir)), 0.0);
  vLighting = 0.25 + 0.75 * diff;
  vColor = aColor;
  gl_Position = uMvp * vec4(aPos, 1.0);
}
)";

  constexpr const char* fsSrc = R"(
#version 330 core
in vec4 vColor;
in float vLighting;
uniform vec4 uColor;
uniform float uUseAssetColor;
out vec4 FragColor;
void main() {
  float useAssetColor = clamp(uUseAssetColor, 0.0, 1.0);
  vec3 baseColor = mix(uColor.rgb, vColor.rgb * uColor.rgb, useAssetColor);
  float alpha = uColor.a * mix(1.0, vColor.a, useAssetColor);
  FragColor = vec4(baseColor * vLighting, alpha);
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
  uNormalMatrixLoc_ = backend_->GetUniformLocation(shader_.Get(), "uNormalMatrix");
  uColorLoc_ = backend_->GetUniformLocation(shader_.Get(), "uColor");
  uLightDirLoc_ = backend_->GetUniformLocation(shader_.Get(), "uLightDir");
  uUseAssetColorLoc_ = backend_->GetUniformLocation(shader_.Get(), "uUseAssetColor");
  materialPipeline_.AddPass(std::make_unique<FlatLitMaterialPass>(*backend_, uColorLoc_));
  if (backend_ != nullptr) {
    materialPipeline_.AddPass(std::make_unique<WireframeMaterialPass>(*backend_));
  }
  return uMvpLoc_ >= 0 && uNormalMatrixLoc_ >= 0 &&
         uColorLoc_ >= 0 && uLightDirLoc_ >= 0 && uUseAssetColorLoc_ >= 0;
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

CadModelRenderFeature::PreparedCpuMesh CadModelRenderFeature::PrepareCpuMesh(const PositionNormalMesh& cpu) {
  PreparedCpuMesh prepared;
  if (cpu.vertices.empty()) {
    return prepared;
  }

  prepared.vertices.reserve(cpu.vertices.size());
  prepared.indices.reserve(cpu.vertices.size());
  std::unordered_map<VertexKey, std::uint32_t, VertexKeyHash> dedup;
  dedup.reserve(cpu.vertices.size());

  for (const auto& v : cpu.vertices) {
    const VertexKey key = ToVertexKey(v);
    const auto it = dedup.find(key);
    if (it != dedup.end()) {
      prepared.indices.push_back(it->second);
      continue;
    }
    const auto idx = static_cast<std::uint32_t>(prepared.vertices.size());
    prepared.vertices.push_back(v);
    dedup.emplace(key, idx);
    prepared.indices.push_back(idx);
  }

  prepared.boundsCenter = ComputeTrimmedBoundsCenterXY(prepared.vertices);
  return prepared;
}

bool CadModelRenderFeature::UploadMesh(const PreparedCpuMesh& cpu, GpuMesh& gpu, IRenderBackend& backend) {
  if (cpu.vertices.empty()) {
    return false;
  }

  const auto& vertices = cpu.vertices;
  const auto& indices = cpu.indices;
  struct PackedVertex {
    glm::vec3 position{0.0F, 0.0F, 0.0F};
    glm::vec3 normal{0.0F, 0.0F, 1.0F};
    std::array<std::uint8_t, 4> color{255, 255, 255, 255};
  };

  std::vector<PackedVertex> packed;
  packed.reserve(vertices.size());
  for (const auto& v : vertices) {
    PackedVertex pv;
    pv.position = v.position;
    pv.normal = v.normal;
    const auto pack = [](const float c) -> std::uint8_t {
      const float clamped = std::clamp(c, 0.0F, 1.0F);
      return static_cast<std::uint8_t>(std::lround(clamped * 255.0F));
    };
    pv.color = {pack(v.color.r), pack(v.color.g), pack(v.color.b), pack(v.color.a)};
    packed.push_back(pv);
  }

  const unsigned int vaoId = backend.CreateVertexArray();
  const unsigned int vboId = backend.CreateBuffer();
  const unsigned int eboId = backend.CreateBuffer();
  if (vaoId == 0 || vboId == 0 || eboId == 0) {
    DestroyMesh(gpu);
    return false;
  }

  gpu.vao.Set(vaoId);
  gpu.vbo.Set(vboId);
  gpu.ebo.Set(eboId);

  backend.BindVertexArray(gpu.vao.Get());
  backend.BindArrayBuffer(gpu.vbo.Get());
  backend.UploadArrayBufferData(packed.size() * sizeof(PackedVertex), packed.data(), false);
  backend.BindElementArrayBuffer(gpu.ebo.Get());
  backend.UploadElementArrayBufferData(indices.size() * sizeof(std::uint32_t), indices.data(), false);

  backend.EnableVertexAttrib(0);
  backend.DefineVertexAttribFloat(0, 3, sizeof(PackedVertex), offsetof(PackedVertex, position));
  backend.EnableVertexAttrib(1);
  backend.DefineVertexAttribFloat(1, 3, sizeof(PackedVertex), offsetof(PackedVertex, normal));
  backend.EnableVertexAttrib(2);
  backend.DefineVertexAttribNormalizedU8(2, 4, sizeof(PackedVertex), offsetof(PackedVertex, color));
  backend.BindVertexArray(0);

  gpu.boundsCenter = cpu.boundsCenter;
  gpu.vertexCount = static_cast<int>(vertices.size());
  gpu.indexCount = static_cast<int>(indices.size());
  return true;
}

void CadModelRenderFeature::DestroyMesh(GpuMesh& gpu) {
  gpu.ebo.Reset();
  gpu.vbo.Reset();
  gpu.vao.Reset();
  gpu.vertexCount = 0;
  gpu.indexCount = 0;
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
      PreparedCpuMesh prepared = pendingIt->second.future.get();
      pendingLoads_.erase(pendingIt);
      if (prepared.vertices.empty()) {
        failedLoads_.insert(assetPath);
        std::cerr << "CAD load failed: " << assetPath << "\n";
        return nullptr;
      }

      GpuMesh gpu;
      if (backend_ == nullptr || !UploadMesh(prepared, gpu, *backend_)) {
        failedLoads_.insert(assetPath);
        std::cerr << "CAD upload failed: " << assetPath << " (vertices=" << prepared.vertices.size() << ")\n";
        return nullptr;
      }

      auto [it, inserted] = meshCache_.emplace(assetPath, std::move(gpu));
      if (!inserted) {
        DestroyMesh(gpu);
      } else {
        std::cerr << "CAD loaded: " << assetPath
                  << " (preparedVertices=" << prepared.vertices.size()
                  << ", uniqueVertices=" << it->second.vertexCount
                  << ", indices=" << it->second.indexCount
                  << ", centerXY=(" << it->second.boundsCenter.x << ", " << it->second.boundsCenter.y << "))\n";
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
              return PreparedCpuMesh{};
            }
            return PrepareCpuMesh(cpu);
          })});
  std::cerr << "CAD loading started: " << assetPath << "\n";
  return nullptr;
}

}  // namespace repulsor3d
