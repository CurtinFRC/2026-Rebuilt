#include "repulsor3d/render/assets/CadMeshAssetPipeline.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "CadMeshImportHelpers.hpp"

namespace repulsor3d {

bool ObjCadMeshImporter::Import(const std::string& resolvedAssetPath, PositionNormalMesh& outMesh) {
  std::ifstream in(resolvedAssetPath);
  if (!in) {
    return false;
  }

  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  outMesh.vertices.clear();

  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line.front() == '#') {
      continue;
    }

    std::istringstream iss(line);
    std::string kind;
    iss >> kind;
    if (kind == "v") {
      glm::vec3 p{};
      if (iss >> p.x >> p.y >> p.z) {
        positions.push_back(p);
      }
    } else if (kind == "vn") {
      glm::vec3 n{};
      if (iss >> n.x >> n.y >> n.z) {
        normals.push_back(cad_import::NormalizeSafe(n));
      }
    } else if (kind == "f") {
      std::vector<int> posIndices;
      std::vector<int> normIndices;
      std::string token;
      while (iss >> token) {
        int pIdx = -1;
        int nIdx = -1;
        if (!cad_import::ParseObjFaceVertex(token, pIdx, nIdx)) {
          continue;
        }
        posIndices.push_back(cad_import::ResolveObjIndex(pIdx, static_cast<int>(positions.size())));
        normIndices.push_back(cad_import::ResolveObjIndex(nIdx, static_cast<int>(normals.size())));
      }

      if (posIndices.size() < 3) {
        continue;
      }

      for (size_t i = 1; i + 1 < posIndices.size(); ++i) {
        const int i0 = posIndices[0];
        const int i1 = posIndices[i];
        const int i2 = posIndices[i + 1];
        if (i0 < 0 || i1 < 0 || i2 < 0 ||
            i0 >= static_cast<int>(positions.size()) ||
            i1 >= static_cast<int>(positions.size()) ||
            i2 >= static_cast<int>(positions.size())) {
          continue;
        }

        const glm::vec3 p0 = positions[static_cast<size_t>(i0)];
        const glm::vec3 p1 = positions[static_cast<size_t>(i1)];
        const glm::vec3 p2 = positions[static_cast<size_t>(i2)];

        glm::vec3 n0 = cad_import::ComputeFallbackNormal(p0, p1, p2);
        glm::vec3 n1 = n0;
        glm::vec3 n2 = n0;

        const int nIdx0 = normIndices[0];
        const int nIdx1 = normIndices[i];
        const int nIdx2 = normIndices[i + 1];
        if (nIdx0 >= 0 && nIdx1 >= 0 && nIdx2 >= 0 &&
            nIdx0 < static_cast<int>(normals.size()) &&
            nIdx1 < static_cast<int>(normals.size()) &&
            nIdx2 < static_cast<int>(normals.size())) {
          n0 = normals[static_cast<size_t>(nIdx0)];
          n1 = normals[static_cast<size_t>(nIdx1)];
          n2 = normals[static_cast<size_t>(nIdx2)];
        }

        outMesh.vertices.push_back({p0, n0});
        outMesh.vertices.push_back({p1, n1});
        outMesh.vertices.push_back({p2, n2});
      }
    }
  }

  return !outMesh.vertices.empty();
}

}  // namespace repulsor3d
