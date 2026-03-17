#include "repulsor3d/render/assets/CadMeshAssetPipeline.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>

namespace repulsor3d {

bool MultiFormatCadMeshImporter::Import(const std::string& resolvedAssetPath, PositionNormalMesh& outMesh) {
  const std::filesystem::path path(resolvedAssetPath);
  std::string ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (ext == ".stl") {
    return stlImporter_.Import(resolvedAssetPath, outMesh);
  }
  if (ext == ".obj") {
    return objImporter_.Import(resolvedAssetPath, outMesh);
  }
  if (ext == ".gltf" || ext == ".glb") {
    return gltfImporter_.Import(resolvedAssetPath, outMesh);
  }

  return stlImporter_.Import(resolvedAssetPath, outMesh) ||
         objImporter_.Import(resolvedAssetPath, outMesh) ||
         gltfImporter_.Import(resolvedAssetPath, outMesh);
}

std::unique_ptr<ICadMeshImporter> CreateDefaultCadMeshImporter() {
  return std::make_unique<MultiFormatCadMeshImporter>();
}

}  // namespace repulsor3d
