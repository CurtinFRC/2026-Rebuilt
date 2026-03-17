#include "repulsor3d/Renderer.hpp"

#include <GL/glew.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <cstddef>
#include <iostream>

namespace repulsor3d {
namespace {

template <typename TVertex>
bool UploadMesh(
    const std::vector<TVertex>& vertices,
    const std::vector<uint32_t>& indices,
    unsigned int& vao,
    unsigned int& vbo,
    unsigned int& ebo,
    int& indexCount,
    const std::size_t uvOffset = 0,
    const bool withUv = false) {
  glGenVertexArrays(1, &vao);
  glGenBuffers(1, &vbo);
  glGenBuffers(1, &ebo);

  if (vao == 0 || vbo == 0 || ebo == 0) {
    return false;
  }

  glBindVertexArray(vao);

  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(
      GL_ARRAY_BUFFER,
      static_cast<GLsizeiptr>(vertices.size() * sizeof(TVertex)),
      vertices.data(),
      GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
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
        reinterpret_cast<void*>(uvOffset));
  }

  glBindVertexArray(0);
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
    const auto& cubeMeshData = geometryProvider_->GetUnitCubeMesh();
    std::vector<VertexP> cubeVerts;
    cubeVerts.reserve(cubeMeshData.positions.size());
    for (const auto& p : cubeMeshData.positions) {
      cubeVerts.push_back({p});
    }

    if (!UploadMesh(
            cubeVerts,
            cubeMeshData.indices,
            cubeMesh_.vao,
            cubeMesh_.vbo,
            cubeMesh_.ebo,
            cubeMesh_.indexCount)) {
      return false;
    }
  }

  {
    const auto& sphereMeshData = geometryProvider_->GetUnitSphereMesh();
    std::vector<VertexP> sphereVerts;
    sphereVerts.reserve(sphereMeshData.positions.size());
    for (const auto& p : sphereMeshData.positions) {
      sphereVerts.push_back({p});
    }

    if (!UploadMesh(
            sphereVerts,
            sphereMeshData.indices,
            sphereMesh_.vao,
            sphereMesh_.vbo,
            sphereMesh_.ebo,
            sphereMesh_.indexCount)) {
      return false;
    }
  }

  {
    const auto& quadMeshData = geometryProvider_->GetUnitQuadUvMesh();
    std::vector<VertexPT> quadVerts;
    quadVerts.reserve(quadMeshData.positions.size());
    for (size_t i = 0; i < quadMeshData.positions.size(); ++i) {
      quadVerts.push_back({quadMeshData.positions[i], quadMeshData.uvs[i]});
    }

    if (!UploadMesh(
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

}  // namespace repulsor3d
