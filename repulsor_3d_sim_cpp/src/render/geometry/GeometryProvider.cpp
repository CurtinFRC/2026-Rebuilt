#include "repulsor3d/render/geometry/GeometryProvider.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

#include <glm/geometric.hpp>

#include "repulsor3d/render/assets/SceneAssetResolver.hpp"

namespace repulsor3d {
namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kEpsilon = 1e-6F;

}  // namespace

DefaultGeometryProvider::DefaultGeometryProvider(const ISceneAssetResolver& assetResolver) : assetResolver_(assetResolver) {}

const PositionIndexedMesh& DefaultGeometryProvider::GetUnitCubeMesh() {
  if (!cubeInitialized_) {
    cubeMesh_ = BuildCubeMesh();
    cubeInitialized_ = true;
  }
  return cubeMesh_;
}

const PositionIndexedMesh& DefaultGeometryProvider::GetUnitSphereMesh() {
  if (!sphereInitialized_) {
    sphereMesh_ = BuildSphereMesh();
    sphereInitialized_ = true;
  }
  return sphereMesh_;
}

const PositionUvIndexedMesh& DefaultGeometryProvider::GetUnitQuadUvMesh() {
  if (!quadInitialized_) {
    quadMesh_ = BuildQuadUvMesh();
    quadInitialized_ = true;
  }
  return quadMesh_;
}

bool DefaultGeometryProvider::GetCadMesh(const std::string& assetPath, PositionNormalMesh& outMesh) {
  const std::string resolved = assetResolver_.ResolveFilePath(assetPath);
  if (resolved.empty()) {
    return false;
  }

  const auto cached = cadMeshCache_.find(resolved);
  if (cached != cadMeshCache_.end()) {
    outMesh = cached->second;
    return true;
  }

  PositionNormalMesh mesh;
  if (!LoadStl(resolved, mesh)) {
    return false;
  }

  cadMeshCache_.emplace(resolved, mesh);
  outMesh = std::move(mesh);
  return true;
}

PositionIndexedMesh DefaultGeometryProvider::BuildCubeMesh() {
  PositionIndexedMesh mesh;
  mesh.positions = {
      {-0.5F, -0.5F, -0.5F},
      {0.5F, -0.5F, -0.5F},
      {0.5F, 0.5F, -0.5F},
      {-0.5F, 0.5F, -0.5F},
      {-0.5F, -0.5F, 0.5F},
      {0.5F, -0.5F, 0.5F},
      {0.5F, 0.5F, 0.5F},
      {-0.5F, 0.5F, 0.5F},
  };
  mesh.indices = {
      0, 1, 2, 2, 3, 0,
      4, 5, 6, 6, 7, 4,
      0, 4, 7, 7, 3, 0,
      1, 5, 6, 6, 2, 1,
      3, 2, 6, 6, 7, 3,
      0, 1, 5, 5, 4, 0,
  };
  return mesh;
}

PositionIndexedMesh DefaultGeometryProvider::BuildSphereMesh() {
  PositionIndexedMesh mesh;
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
      mesh.positions.push_back({sp * ct, sp * st, cp});
    }
  }

  const int stride = segments + 1;
  for (int r = 0; r < rings; ++r) {
    const int base0 = r * stride;
    const int base1 = (r + 1) * stride;
    for (int s = 0; s < segments; ++s) {
      const std::uint32_t a = static_cast<std::uint32_t>(base0 + s);
      const std::uint32_t b = static_cast<std::uint32_t>(base1 + s);
      const std::uint32_t c = static_cast<std::uint32_t>(base1 + s + 1);
      const std::uint32_t d = static_cast<std::uint32_t>(base0 + s + 1);
      mesh.indices.insert(mesh.indices.end(), {a, b, c, a, c, d});
    }
  }

  return mesh;
}

PositionUvIndexedMesh DefaultGeometryProvider::BuildQuadUvMesh() {
  PositionUvIndexedMesh mesh;
  mesh.positions = {
      {0.0F, 0.0F, 0.0F},
      {1.0F, 0.0F, 0.0F},
      {1.0F, 1.0F, 0.0F},
      {0.0F, 1.0F, 0.0F},
  };
  mesh.uvs = {
      {0.0F, 0.0F},
      {1.0F, 0.0F},
      {1.0F, 1.0F},
      {0.0F, 1.0F},
  };
  mesh.indices = {0, 1, 2, 2, 3, 0};
  return mesh;
}

glm::vec3 DefaultGeometryProvider::NormalizeSafe(const glm::vec3& v) {
  const float len = glm::length(v);
  if (len <= kEpsilon) {
    return {0.0F, 0.0F, 1.0F};
  }
  return v / len;
}

glm::vec3 DefaultGeometryProvider::ComputeFallbackNormal(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
  return NormalizeSafe(glm::cross(b - a, c - a));
}

bool DefaultGeometryProvider::ParseBinaryStl(const std::string& filePath, PositionNormalMesh& outMesh) {
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

  std::uint32_t triCount = 0;
  in.read(reinterpret_cast<char*>(&triCount), sizeof(std::uint32_t));
  if (!in) {
    return false;
  }

  const std::uint64_t expected = 84ULL + static_cast<std::uint64_t>(triCount) * 50ULL;
  if (expected != static_cast<std::uint64_t>(fileSize)) {
    return false;
  }

  outMesh.vertices.clear();
  outMesh.vertices.reserve(static_cast<size_t>(triCount) * 3);

  for (std::uint32_t i = 0; i < triCount; ++i) {
    float nx = 0.0F;
    float ny = 0.0F;
    float nz = 1.0F;
    glm::vec3 p[3];

    in.read(reinterpret_cast<char*>(&nx), sizeof(float));
    in.read(reinterpret_cast<char*>(&ny), sizeof(float));
    in.read(reinterpret_cast<char*>(&nz), sizeof(float));

    for (auto& vertex : p) {
      in.read(reinterpret_cast<char*>(&vertex.x), sizeof(float));
      in.read(reinterpret_cast<char*>(&vertex.y), sizeof(float));
      in.read(reinterpret_cast<char*>(&vertex.z), sizeof(float));
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

bool DefaultGeometryProvider::ParseAsciiStl(const std::string& filePath, PositionNormalMesh& outMesh) {
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
    for (auto& vertex : p) {
      if (!(in >> token) || token != "vertex") {
        return false;
      }
      if (!(in >> vertex.x >> vertex.y >> vertex.z)) {
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

bool DefaultGeometryProvider::LoadStl(const std::string& filePath, PositionNormalMesh& outMesh) {
  if (ParseBinaryStl(filePath, outMesh)) {
    return true;
  }
  return ParseAsciiStl(filePath, outMesh);
}

}  // namespace repulsor3d
