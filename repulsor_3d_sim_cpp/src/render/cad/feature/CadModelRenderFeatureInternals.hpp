#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "repulsor3d/render/CadModelRenderFeature.hpp"
#include "repulsor3d/render/cad/CadPolicies.hpp"

namespace repulsor3d::cadfeature {

struct PreparedCpuMeshBlob {
  struct LodBlob {
    std::vector<PositionNormalMesh::Vertex> vertices;
    std::vector<std::uint32_t> indices;
  };
  std::vector<LodBlob> lods;
  glm::vec3 boundsCenter{0.0F, 0.0F, 0.0F};
  float boundsRadius = 1.0F;
  float roughnessHint = -1.0F;
  float metallicHint = -1.0F;
};

struct IndexedMesh {
  std::vector<PositionNormalMesh::Vertex> vertices;
  std::vector<std::uint32_t> indices;
};

struct PackedVertex {
  glm::vec3 position{0.0F, 0.0F, 0.0F};
  glm::vec3 normal{0.0F, 0.0F, 1.0F};
  std::array<std::uint8_t, 4> color{255, 255, 255, 255};
};

struct InstanceGpuData {
  glm::vec4 modelRow0{1.0F, 0.0F, 0.0F, 0.0F};
  glm::vec4 modelRow1{0.0F, 1.0F, 0.0F, 0.0F};
  glm::vec4 modelRow2{0.0F, 0.0F, 1.0F, 0.0F};
  glm::vec4 modelRow3{0.0F, 0.0F, 0.0F, 1.0F};
};

std::uint64_t BuildBroadphaseFingerprint(
    const std::vector<glm::vec3>& centers,
    const std::vector<float>& radii,
    float cellSizeMeters);

bool TryLoadPreparedCpuMeshCache(
    const std::string& assetPath,
    const cad::CadLodPolicy& policy,
    PreparedCpuMeshBlob& outPrepared);

void StorePreparedCpuMeshCache(
    const std::string& assetPath,
    const cad::CadLodPolicy& policy,
    const PreparedCpuMeshBlob& prepared);

glm::vec3 ComputeTrimmedBoundsCenterXY(const std::vector<PositionNormalMesh::Vertex>& vertices);
float ComputeRadiusFromCenter(const std::vector<PositionNormalMesh::Vertex>& vertices, const glm::vec3& center);
IndexedMesh BuildIndexedMesh(const PositionNormalMesh& mesh);
void RemoveDegenerateTriangles(IndexedMesh& mesh);
std::vector<IndexedMesh> BuildLodChain(const IndexedMesh& baseMesh, const cad::CadLodPolicy& policy);
IndexedMesh BuildShadowProxyMesh(const std::vector<IndexedMesh>& lods, const cad::CadLodPolicy& policy);
float ComputeScreenSpaceRadiusPixels(
    const glm::vec3& center,
    float radius,
    const glm::mat4& mvp,
    int viewportWidth,
    int viewportHeight);

InstanceGpuData ToInstanceData(const glm::mat4& model);
std::vector<PackedVertex> PackVerticesForGpu(const std::vector<PositionNormalMesh::Vertex>& vertices);

}  // namespace repulsor3d::cadfeature
