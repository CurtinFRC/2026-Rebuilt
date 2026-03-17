#include "repulsor3d/render/assets/CadMeshAssetPipeline.hpp"

#include <array>
#include <cstdint>
#include <fstream>
#include <string>

#include "CadMeshImportHelpers.hpp"

namespace repulsor3d {

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
    glm::vec3 normal = cad_import::NormalizeSafe(rawNormal);
    if (glm::length(rawNormal) <= 1e-6F) {
      normal = cad_import::ComputeFallbackNormal(p[0], p[1], p[2]);
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
    glm::vec3 normal = cad_import::NormalizeSafe(rawNormal);
    if (glm::length(rawNormal) <= 1e-6F) {
      normal = cad_import::ComputeFallbackNormal(p[0], p[1], p[2]);
    }

    outMesh.vertices.push_back({p[0], normal});
    outMesh.vertices.push_back({p[1], normal});
    outMesh.vertices.push_back({p[2], normal});
  }

  return !outMesh.vertices.empty();
}

}  // namespace repulsor3d
