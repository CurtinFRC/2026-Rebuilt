#include "repulsor3d/render/geometry/GeometryProvider.hpp"

#include <algorithm>
#include <cmath>

#include "repulsor3d/render/assets/CadMeshAssetPipeline.hpp"
#include "repulsor3d/render/assets/SceneAssetResolver.hpp"

namespace repulsor3d {
namespace {

constexpr float kPi = 3.14159265358979323846F;

}  // namespace

DefaultGeometryProvider::DefaultGeometryProvider(const ISceneAssetResolver& assetResolver)
    : assetResolver_(assetResolver), cadAssetPipeline_(std::make_unique<CadMeshAssetPipeline>(assetResolver_)) {}

DefaultGeometryProvider::~DefaultGeometryProvider() = default;

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
  if (cadAssetPipeline_ == nullptr) {
    return false;
  }
  return cadAssetPipeline_->Load(assetPath, outMesh);
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

}  // namespace repulsor3d
