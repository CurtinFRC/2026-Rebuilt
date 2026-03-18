#pragma once

#include <memory>
#include <string>

#include "repulsor3d/render/geometry/GeometryProvider.hpp"

namespace repulsor3d {

class IMeshProvider {
 public:
  virtual ~IMeshProvider() = default;
  virtual bool LoadPositionNormalMesh(const std::string& assetPath, PositionNormalMesh& outMesh) = 0;
};

class GeometryMeshProvider final : public IMeshProvider {
 public:
  explicit GeometryMeshProvider(IGeometryProvider& geometryProvider) : geometryProvider_(geometryProvider) {}

  bool LoadPositionNormalMesh(const std::string& assetPath, PositionNormalMesh& outMesh) override;

 private:
  IGeometryProvider& geometryProvider_;
};

std::unique_ptr<IMeshProvider> CreateDefaultMeshProvider(IGeometryProvider& geometryProvider);

}  // namespace repulsor3d
