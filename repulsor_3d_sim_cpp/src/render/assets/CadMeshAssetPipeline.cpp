#include "repulsor3d/render/assets/CadMeshAssetPipeline.hpp"

#include <array>
#include <cstdint>
#include <fstream>
#include <string>

#include <glm/geometric.hpp>

#include "repulsor3d/render/assets/SceneAssetResolver.hpp"

namespace repulsor3d {
namespace {

constexpr float kEpsilon = 1e-6F;

glm::vec3 NormalizeSafe(const glm::vec3& v) {
  const float len = glm::length(v);
  if (len <= kEpsilon) {
    return {0.0F, 0.0F, 1.0F};
  }
  return v / len;
}

glm::vec3 ComputeFallbackNormal(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
  return NormalizeSafe(glm::cross(b - a, c - a));
}

}  // namespace

bool StlCadMeshImporter::Import(const std::string& resolvedAssetPath, PositionNormalMesh& outMesh) {
  if (ParseBinaryStl(resolvedAssetPath, outMesh)) {
    return true;
  }
  return ParseAsciiStl(resolvedAssetPath, outMesh);
}

bool StlCadMeshImporter::ParseBinaryStl(const std::string& filePath, PositionNormalMesh& outMesh) {
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

bool StlCadMeshImporter::ParseAsciiStl(const std::string& filePath, PositionNormalMesh& outMesh) {
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

void PassThroughCadMeshCooker::Cook(PositionNormalMesh& /*mesh*/) {}

bool InMemoryCadMeshCache::TryLoad(const std::string& key, PositionNormalMesh& outMesh) const {
  const auto it = cache_.find(key);
  if (it == cache_.end()) {
    return false;
  }
  outMesh = it->second;
  return true;
}

void InMemoryCadMeshCache::Store(const std::string& key, const PositionNormalMesh& mesh) {
  cache_[key] = mesh;
}

CadMeshAssetPipeline::CadMeshAssetPipeline(
    const ISceneAssetResolver& resolver,
    std::unique_ptr<ICadMeshImporter> importer,
    std::unique_ptr<ICadMeshCooker> cooker,
    std::unique_ptr<ICadMeshCache> cache)
    : resolver_(resolver), importer_(std::move(importer)), cooker_(std::move(cooker)), cache_(std::move(cache)) {}

bool CadMeshAssetPipeline::Load(const std::string& assetPath, PositionNormalMesh& outMesh) {
  if (importer_ == nullptr || cooker_ == nullptr || cache_ == nullptr) {
    return false;
  }

  const std::string resolved = resolver_.ResolveFilePath(assetPath);
  if (resolved.empty()) {
    return false;
  }

  if (cache_->TryLoad(resolved, outMesh)) {
    return true;
  }

  PositionNormalMesh mesh;
  if (!importer_->Import(resolved, mesh)) {
    return false;
  }

  cooker_->Cook(mesh);
  cache_->Store(resolved, mesh);
  outMesh = std::move(mesh);
  return true;
}

}  // namespace repulsor3d

