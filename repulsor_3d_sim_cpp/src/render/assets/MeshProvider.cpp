#include "repulsor3d/render/assets/MeshProvider.hpp"

namespace repulsor3d {

bool GeometryMeshProvider::LoadPositionNormalMesh(const std::string& assetPath, PositionNormalMesh& outMesh) {
  return geometryProvider_.GetCadMesh(assetPath, outMesh);
}

std::unique_ptr<IMeshProvider> CreateDefaultMeshProvider(IGeometryProvider& geometryProvider) {
  return std::make_unique<GeometryMeshProvider>(geometryProvider);
}

}  // namespace repulsor3d
