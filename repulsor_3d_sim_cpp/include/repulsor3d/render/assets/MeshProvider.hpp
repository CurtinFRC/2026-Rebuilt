#pragma once

#include <memory>
#include <string>
#include <vector>

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

class ProceduralMeshProvider final : public IMeshProvider {
 public:
  explicit ProceduralMeshProvider(IGeometryProvider& geometryProvider) : geometryProvider_(geometryProvider) {}
  bool LoadPositionNormalMesh(const std::string& assetPath, PositionNormalMesh& outMesh) override;

 private:
  IGeometryProvider& geometryProvider_;
};

class StreamingAliasMeshProvider final : public IMeshProvider {
 public:
  explicit StreamingAliasMeshProvider(std::unique_ptr<IMeshProvider> delegate) : delegate_(std::move(delegate)) {}
  bool LoadPositionNormalMesh(const std::string& assetPath, PositionNormalMesh& outMesh) override;

 private:
  std::unique_ptr<IMeshProvider> delegate_;
};

class CompositeMeshProvider final : public IMeshProvider {
 public:
  void Add(std::unique_ptr<IMeshProvider> provider);
  bool LoadPositionNormalMesh(const std::string& assetPath, PositionNormalMesh& outMesh) override;

 private:
  std::vector<std::unique_ptr<IMeshProvider>> providers_;
};

std::unique_ptr<IMeshProvider> CreateDefaultMeshProvider(IGeometryProvider& geometryProvider);

}  // namespace repulsor3d
