#include "repulsor3d/Renderer.hpp"

#include <GL/glew.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <algorithm>
#include <cstddef>
#include <iostream>

namespace repulsor3d {
namespace {

template <typename TVertex>
bool UploadMesh(
    IRenderBackend& backend,
    const std::vector<TVertex>& vertices,
    const std::vector<uint32_t>& indices,
    GlVertexArrayHandle& vao,
    GlBufferHandle& vbo,
    GlBufferHandle& ebo,
    int& indexCount,
    const std::size_t uvOffset = 0,
    const bool withUv = false) {
  const unsigned int vaoId = backend.CreateVertexArray();
  const unsigned int vboId = backend.CreateBuffer();
  const unsigned int eboId = backend.CreateBuffer();

  if (vaoId == 0 || vboId == 0 || eboId == 0) {
    return false;
  }

  vao.Set(vaoId);
  vbo.Set(vboId);
  ebo.Set(eboId);

  backend.BindVertexArray(vao.Get());

  backend.BindArrayBuffer(vbo.Get());
  backend.UploadArrayBufferData(vertices.size() * sizeof(TVertex), vertices.data(), false);

  backend.BindElementArrayBuffer(ebo.Get());
  backend.UploadElementArrayBufferData(indices.size() * sizeof(uint32_t), indices.data(), false);

  backend.EnableVertexAttrib(0);
  backend.DefineVertexAttribFloat(0, 3, sizeof(TVertex), 0);

  if (withUv) {
    backend.EnableVertexAttrib(1);
    backend.DefineVertexAttribFloat(1, 2, sizeof(TVertex), uvOffset);
  }

  backend.BindVertexArray(0);
  indexCount = static_cast<int>(indices.size());
  return true;
}

}  // namespace

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

  constexpr const char* skyVs = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 uMvp;
out vec3 vDir;
void main() {
  vDir = aPos;
  gl_Position = uMvp * vec4(aPos, 1.0);
}
)";

  constexpr const char* skyFs = R"(
#version 330 core
in vec3 vDir;
uniform vec3 uSkyZenith;
uniform vec3 uSkyHorizon;
uniform vec3 uSkyGround;
uniform vec3 uArenaGlow;
uniform float uTime;
out vec4 FragColor;

float Hash12(vec2 p) {
  float h = dot(p, vec2(127.1, 311.7));
  return fract(sin(h) * 43758.5453123);
}

void main() {
  vec3 dir = normalize(vDir);
  float up = clamp(dir.z * 0.5 + 0.5, 0.0, 1.0);
  float horizon = exp(-abs(dir.z) * 14.0);
  vec3 col = mix(uSkyGround, uSkyZenith, smoothstep(0.0, 1.0, up));
  col = mix(col, uSkyHorizon, clamp(horizon * 0.75, 0.0, 1.0));

  // Stylized aurora streaks to add arena atmosphere depth.
  float drift = uTime * 0.06;
  float swirl = sin((dir.x * 5.2 + dir.y * 7.1) + dir.z * 10.0 + drift * 2.7);
  float arc = smoothstep(0.35, 0.95, up) * (0.5 + 0.5 * swirl);
  vec3 auroraA = vec3(0.08, 0.38, 0.62);
  vec3 auroraB = vec3(0.09, 0.22, 0.44);
  col += mix(auroraA, auroraB, 0.5 + 0.5 * dir.x) * arc * 0.24;

  // Sparse stars on upper hemisphere so it still reads as an arena night sky.
  float dirLen = max(length(dir.xy), 1e-4);
  vec2 starUv = (dir.xy / dirLen) * (1.0 + abs(dir.z)) * 420.0 + vec2(drift * 12.0, -drift * 9.0);
  float starNoise = Hash12(floor(starUv));
  float twinkle = 0.75 + 0.25 * sin(uTime * 0.9 + starNoise * 12.0);
  float starGate = step(0.9965, starNoise) * smoothstep(0.55, 0.92, up);
  col += vec3(1.0, 0.98, 0.93) * starGate * twinkle * 0.5;

  // Soft arena glow around horizon.
  col += uArenaGlow * pow(clamp(1.0 - abs(dir.z), 0.0, 1.0), 5.0) * 0.55;

  // Distant moon disk for additional depth and orientation.
  vec3 moonDir = normalize(vec3(-0.45, 0.35, 0.82));
  float moonDot = dot(dir, moonDir);
  float moon = smoothstep(0.992, 0.9985, moonDot);
  col += vec3(0.95, 0.98, 1.0) * moon * 0.85;

  // Very soft procedural cloud bands.
  float cloudBase = sin(dir.x * 12.0 + dir.y * 8.0 + uTime * 0.07) * 0.5 + 0.5;
  float cloudFine = sin(dir.x * 33.0 - dir.y * 19.0 - uTime * 0.13) * 0.5 + 0.5;
  float cloudMask = smoothstep(0.62, 0.95, cloudBase * 0.65 + cloudFine * 0.35) * smoothstep(0.35, 0.92, up);
  col = mix(col, col + vec3(0.10, 0.14, 0.18), cloudMask * 0.24);

  // Subtle chromatic fringe near horizon for cinematic bloom feel.
  float fringe = pow(clamp(1.0 - abs(dir.z), 0.0, 1.0), 7.0);
  col += vec3(0.05, 0.11, 0.18) * fringe * 0.55;
  FragColor = vec4(col, 1.0);
}
)";

  constexpr const char* postVs = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aUv;
out vec2 vUv;
void main() {
  vUv = aUv;
  vec2 ndc = aPos.xy * 2.0 - 1.0;
  gl_Position = vec4(ndc, 0.0, 1.0);
}
)";

  constexpr const char* bloomExtractFs = R"(
#version 330 core
in vec2 vUv;
uniform sampler2D uSceneTex;
uniform float uThreshold;
out vec4 FragColor;
void main() {
  vec3 scene = texture(uSceneTex, vUv).rgb;
  float luma = dot(scene, vec3(0.2126, 0.7152, 0.0722));
  float bright = smoothstep(uThreshold - 0.10, uThreshold + 0.15, luma);
  FragColor = vec4(scene * bright, 1.0);
}
)";

  constexpr const char* bloomBlurFs = R"(
#version 330 core
in vec2 vUv;
uniform sampler2D uSourceTex;
uniform vec3 uDirection;
uniform vec3 uTexelSize;
out vec4 FragColor;
void main() {
  vec2 stepUv = uDirection.xy * uTexelSize.xy;
  vec3 color = texture(uSourceTex, vUv).rgb * 0.227027;
  color += texture(uSourceTex, vUv + stepUv * 1.384615).rgb * 0.316216;
  color += texture(uSourceTex, vUv - stepUv * 1.384615).rgb * 0.316216;
  color += texture(uSourceTex, vUv + stepUv * 3.230769).rgb * 0.070270;
  color += texture(uSourceTex, vUv - stepUv * 3.230769).rgb * 0.070270;
  FragColor = vec4(color, 1.0);
}
)";

  constexpr const char* bloomCompositeFs = R"(
#version 330 core
in vec2 vUv;
uniform sampler2D uSceneTex;
uniform sampler2D uBloomTex;
uniform float uBloomStrength;
out vec4 FragColor;
void main() {
  vec3 scene = texture(uSceneTex, vUv).rgb;
  vec3 bloom = texture(uBloomTex, vUv).rgb;
  vec3 color = scene + bloom * uBloomStrength;
  FragColor = vec4(color, 1.0);
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

  vs = CompileShader(GL_VERTEX_SHADER, skyVs);
  fs = CompileShader(GL_FRAGMENT_SHADER, skyFs);
  if (vs == 0 || fs == 0 || !LinkShader(skyShader_, vs, fs)) {
    return false;
  }
  glDeleteShader(vs);
  glDeleteShader(fs);

  vs = CompileShader(GL_VERTEX_SHADER, postVs);
  fs = CompileShader(GL_FRAGMENT_SHADER, bloomExtractFs);
  if (vs == 0 || fs == 0 || !LinkShader(bloomExtractShader_, vs, fs)) {
    return false;
  }
  glDeleteShader(vs);
  glDeleteShader(fs);

  vs = CompileShader(GL_VERTEX_SHADER, postVs);
  fs = CompileShader(GL_FRAGMENT_SHADER, bloomBlurFs);
  if (vs == 0 || fs == 0 || !LinkShader(bloomBlurShader_, vs, fs)) {
    return false;
  }
  glDeleteShader(vs);
  glDeleteShader(fs);

  vs = CompileShader(GL_VERTEX_SHADER, postVs);
  fs = CompileShader(GL_FRAGMENT_SHADER, bloomCompositeFs);
  if (vs == 0 || fs == 0 || !LinkShader(bloomCompositeShader_, vs, fs)) {
    return false;
  }
  glDeleteShader(vs);
  glDeleteShader(fs);

  return true;
}

bool Renderer::CreateMeshes() {
  {
    const auto& cubeMeshData = geometryProvider_->GetUnitCubeMesh();
    std::vector<VertexP> cubeVerts;
    cubeVerts.reserve(cubeMeshData.positions.size());
    for (const auto& p : cubeMeshData.positions) {
      cubeVerts.push_back({p});
    }

    if (!UploadMesh(
            *backend_,
            cubeVerts,
            cubeMeshData.indices,
            cubeMesh_.vao,
            cubeMesh_.vbo,
            cubeMesh_.ebo,
            cubeMesh_.indexCount)) {
      return false;
    }
    resourceManager_.Register(ResourceClass::VertexArray);
    resourceManager_.Register(ResourceClass::Buffer);
    resourceManager_.Register(ResourceClass::Buffer);
  }

  {
    const auto& sphereMeshData = geometryProvider_->GetUnitSphereMesh();
    std::vector<VertexP> sphereVerts;
    sphereVerts.reserve(sphereMeshData.positions.size());
    for (const auto& p : sphereMeshData.positions) {
      sphereVerts.push_back({p});
    }

    if (!UploadMesh(
            *backend_,
            sphereVerts,
            sphereMeshData.indices,
            sphereMesh_.vao,
            sphereMesh_.vbo,
            sphereMesh_.ebo,
            sphereMesh_.indexCount)) {
      return false;
    }
    resourceManager_.Register(ResourceClass::VertexArray);
    resourceManager_.Register(ResourceClass::Buffer);
    resourceManager_.Register(ResourceClass::Buffer);
  }

  {
    const auto& quadMeshData = geometryProvider_->GetUnitQuadUvMesh();
    std::vector<VertexPT> quadVerts;
    quadVerts.reserve(quadMeshData.positions.size());
    for (size_t i = 0; i < quadMeshData.positions.size(); ++i) {
      quadVerts.push_back({quadMeshData.positions[i], quadMeshData.uvs[i]});
    }

    if (!UploadMesh(
            *backend_,
            quadVerts,
            quadMeshData.indices,
            quadMesh_.vao,
            quadMesh_.vbo,
            quadMesh_.ebo,
            quadMesh_.indexCount,
            offsetof(VertexPT, uv),
            true)) {
      return false;
    }
    resourceManager_.Register(ResourceClass::VertexArray);
    resourceManager_.Register(ResourceClass::Buffer);
    resourceManager_.Register(ResourceClass::Buffer);
  }

  return true;
}

bool Renderer::CreateFieldTexture() {
  if (cfg_.fieldImagePath.empty()) {
    return false;
  }

  const std::string resolved = assetResolver_->ResolveFilePath(cfg_.fieldImagePath);
  if (resolved.empty()) {
    return false;
  }

  stbi_set_flip_vertically_on_load(0);
  int width = 0;
  int height = 0;
  int channels = 0;
  unsigned char* pixels = stbi_load(resolved.c_str(), &width, &height, &channels, STBI_rgb_alpha);
  if (pixels == nullptr) {
    return false;
  }

  const unsigned int textureId = backend_->CreateTexture2D();
  fieldTexture_.Set(textureId);
  resourceManager_.Register(
      ResourceClass::Texture,
      "renderer.field_texture",
      static_cast<std::size_t>(std::max(0, width)) * static_cast<std::size_t>(std::max(0, height)) * 4U);
  backend_->BindTexture2D(fieldTexture_.Get());
  backend_->SetTexture2DLinearMipmapClamp();
  backend_->UploadTexture2DRgba8(width, height, pixels);
  backend_->GenerateTexture2DMipmaps();
  backend_->BindTexture2D(0);
  stbi_image_free(pixels);
  return true;
}

void Renderer::DestroyShader(Shader& shader) const {
  shader.program.Reset();
}

void Renderer::DestroyMesh(Mesh& mesh) {
  if (mesh.ebo.Get() != 0) {
    resourceManager_.Release(ResourceClass::Buffer);
  }
  if (mesh.vbo.Get() != 0) {
    resourceManager_.Release(ResourceClass::Buffer);
  }
  if (mesh.vao.Get() != 0) {
    resourceManager_.Release(ResourceClass::VertexArray);
  }
  mesh.ebo.Reset();
  mesh.vbo.Reset();
  mesh.vao.Reset();
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
    shader.program.Set(program);
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

}  // namespace repulsor3d
