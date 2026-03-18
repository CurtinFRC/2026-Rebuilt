#include "repulsor3d/render/assets/MeshProvider.hpp"

#include <algorithm>

namespace repulsor3d {
namespace {

bool StartsWith(const std::string& value, const std::string& prefix) {
  return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0;
}

PositionNormalMesh FromIndexedMesh(const PositionIndexedMesh& indexed) {
  PositionNormalMesh out;
  out.vertices.reserve(indexed.indices.size());
  for (const auto index : indexed.indices) {
    if (index >= indexed.positions.size()) {
      continue;
    }
    PositionNormalMesh::Vertex vertex;
    vertex.position = indexed.positions[index];
    out.vertices.push_back(vertex);
  }
  return out;
}

}  // namespace

bool GeometryMeshProvider::LoadPositionNormalMesh(const std::string& assetPath, PositionNormalMesh& outMesh) {
  return geometryProvider_.GetCadMesh(assetPath, outMesh);
}

bool ProceduralMeshProvider::LoadPositionNormalMesh(const std::string& assetPath, PositionNormalMesh& outMesh) {
  if (!StartsWith(assetPath, "primitive://")) {
    return false;
  }

  const std::string primitiveType = assetPath.substr(std::string("primitive://").size());
  if (primitiveType == "cube") {
    outMesh = FromIndexedMesh(geometryProvider_.GetUnitCubeMesh());
    return !outMesh.vertices.empty();
  }
  if (primitiveType == "sphere") {
    outMesh = FromIndexedMesh(geometryProvider_.GetUnitSphereMesh());
    return !outMesh.vertices.empty();
  }
  if (primitiveType == "quad") {
    PositionNormalMesh quad;
    const auto& src = geometryProvider_.GetUnitQuadUvMesh();
    quad.vertices.reserve(src.indices.size());
    for (const auto index : src.indices) {
      if (index >= src.positions.size()) {
        continue;
      }
      PositionNormalMesh::Vertex v;
      v.position = src.positions[index];
      quad.vertices.push_back(v);
    }
    outMesh = std::move(quad);
    return !outMesh.vertices.empty();
  }

  return false;
}

bool StreamingAliasMeshProvider::LoadPositionNormalMesh(const std::string& assetPath, PositionNormalMesh& outMesh) {
  if (delegate_ == nullptr) {
    return false;
  }

  if (StartsWith(assetPath, "stream://")) {
    const std::string mapped = assetPath.substr(std::string("stream://").size());
    if (mapped.empty()) {
      return false;
    }
    return delegate_->LoadPositionNormalMesh(mapped, outMesh);
  }

  return delegate_->LoadPositionNormalMesh(assetPath, outMesh);
}

void CompositeMeshProvider::Add(std::unique_ptr<IMeshProvider> provider) {
  if (provider != nullptr) {
    providers_.push_back(std::move(provider));
  }
}

bool CompositeMeshProvider::LoadPositionNormalMesh(const std::string& assetPath, PositionNormalMesh& outMesh) {
  for (const auto& provider : providers_) {
    if (provider == nullptr) {
      continue;
    }
    if (provider->LoadPositionNormalMesh(assetPath, outMesh)) {
      return true;
    }
  }
  return false;
}

std::unique_ptr<IMeshProvider> CreateDefaultMeshProvider(IGeometryProvider& geometryProvider) {
  auto composite = std::make_unique<CompositeMeshProvider>();
  composite->Add(std::make_unique<ProceduralMeshProvider>(geometryProvider));
  composite->Add(std::make_unique<StreamingAliasMeshProvider>(std::make_unique<GeometryMeshProvider>(geometryProvider)));
  return composite;
}

}  // namespace repulsor3d
