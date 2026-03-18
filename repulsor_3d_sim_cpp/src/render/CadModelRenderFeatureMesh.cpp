#include "repulsor3d/render/CadModelRenderFeature.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "repulsor3d/render/cad/CadPolicies.hpp"
#include "cad/feature/CadModelRenderFeatureInternals.hpp"

namespace repulsor3d {

using cad::CadLodPolicy;
using cad::LoadCadLodPolicy;
using cadfeature::BuildIndexedMesh;
using cadfeature::BuildLodChain;
using cadfeature::BuildShadowProxyMesh;
using cadfeature::ComputeRadiusFromCenter;
using cadfeature::ComputeScreenSpaceRadiusPixels;
using cadfeature::ComputeTrimmedBoundsCenterXY;
using cadfeature::IndexedMesh;
using cadfeature::InstanceGpuData;
using cadfeature::PackedVertex;
using cadfeature::PackVerticesForGpu;
using cadfeature::RemoveDegenerateTriangles;

CadModelRenderFeature::PreparedCpuMesh CadModelRenderFeature::PrepareCpuMesh(const PositionNormalMesh& cpu) {
  PreparedCpuMesh prepared;
  if (cpu.vertices.empty()) {
    return prepared;
  }

  const CadLodPolicy& lodPolicy = LoadCadLodPolicy();
  IndexedMesh indexed = BuildIndexedMesh(cpu);
  RemoveDegenerateTriangles(indexed);
  if (indexed.vertices.empty() || indexed.indices.empty()) {
    return prepared;
  }

  const glm::vec3 center = ComputeTrimmedBoundsCenterXY(indexed.vertices);
  const float radius = ComputeRadiusFromCenter(indexed.vertices, center);
  std::vector<IndexedMesh> lodMeshes = BuildLodChain(indexed, lodPolicy);
  IndexedMesh shadowProxy = BuildShadowProxyMesh(lodMeshes, lodPolicy);
  prepared.lods.reserve(lodMeshes.size());
  for (auto& lod : lodMeshes) {
    if (lod.vertices.empty() || lod.indices.empty()) {
      continue;
    }
    PreparedCpuMesh::LodCpu outLod;
    outLod.vertices = std::move(lod.vertices);
    outLod.indices = std::move(lod.indices);
    prepared.lods.push_back(std::move(outLod));
  }
  if (!shadowProxy.vertices.empty() && !shadowProxy.indices.empty()) {
    PreparedCpuMesh::LodCpu outShadow;
    outShadow.vertices = std::move(shadowProxy.vertices);
    outShadow.indices = std::move(shadowProxy.indices);
    prepared.shadowProxy = std::move(outShadow);
  }

  prepared.boundsCenter = center;
  prepared.boundsRadius = radius;
  prepared.materialHints = cpu.materialHints;
  return prepared;
}

bool CadModelRenderFeature::UploadMesh(const PreparedCpuMesh& cpu, GpuMesh& gpu, IRenderBackend& backend) {
  DestroyMesh(gpu);
  if (cpu.lods.empty()) {
    return false;
  }

  gpu.lods.reserve(cpu.lods.size());
  for (const auto& cpuLod : cpu.lods) {
    if (cpuLod.vertices.empty()) {
      continue;
    }

    GpuMesh::LodGpu gpuLod;
    if (!UploadSingleLod(cpuLod, gpuLod, backend)) {
      DestroyMesh(gpu);
      return false;
    }
    gpu.lods.push_back(std::move(gpuLod));
  }
  if (cpu.shadowProxy.has_value()) {
    GpuMesh::LodGpu gpuShadow;
    if (!UploadSingleLod(cpu.shadowProxy.value(), gpuShadow, backend)) {
      DestroyMesh(gpu);
      return false;
    }
    gpu.shadowProxy = std::move(gpuShadow);
  }

  gpu.boundsCenter = cpu.boundsCenter;
  gpu.boundsRadius = cpu.boundsRadius;
  gpu.materialHints = cpu.materialHints;
  return !gpu.lods.empty();
}

bool CadModelRenderFeature::UploadSingleLod(
    const PreparedCpuMesh::LodCpu& cpuLod,
    GpuMesh::LodGpu& gpuLod,
    IRenderBackend& backend) {
  const unsigned int vao = backend.CreateVertexArray();
  const unsigned int vbo = backend.CreateBuffer();
  const unsigned int ebo = backend.CreateBuffer();
  const unsigned int instanceVbo = backend.CreateBuffer();
  if (vao == 0 || vbo == 0 || ebo == 0 || instanceVbo == 0) {
    return false;
  }

  gpuLod.vao.Set(vao);
  gpuLod.vbo.Set(vbo);
  gpuLod.ebo.Set(ebo);
  gpuLod.instanceVbo.Set(instanceVbo);

  const std::vector<PackedVertex> packed = PackVerticesForGpu(cpuLod.vertices);
  backend.BindVertexArray(gpuLod.vao.Get());
  backend.BindArrayBuffer(gpuLod.vbo.Get());
  backend.UploadArrayBufferData(packed.size() * sizeof(PackedVertex), packed.data(), false);
  backend.BindElementArrayBuffer(gpuLod.ebo.Get());
  backend.UploadElementArrayBufferData(cpuLod.indices.size() * sizeof(std::uint32_t), cpuLod.indices.data(), false);

  backend.EnableVertexAttrib(0);
  backend.DefineVertexAttribFloat(0, 3, sizeof(PackedVertex), offsetof(PackedVertex, position));
  backend.EnableVertexAttrib(1);
  backend.DefineVertexAttribFloat(1, 3, sizeof(PackedVertex), offsetof(PackedVertex, normal));
  backend.EnableVertexAttrib(2);
  backend.DefineVertexAttribNormalizedU8(2, 4, sizeof(PackedVertex), offsetof(PackedVertex, color));

  backend.BindArrayBuffer(gpuLod.instanceVbo.Get());
  backend.UploadArrayBufferData(sizeof(InstanceGpuData), nullptr, true);
  backend.EnableVertexAttrib(3);
  backend.DefineVertexAttribFloat(3, 4, sizeof(InstanceGpuData), offsetof(InstanceGpuData, modelRow0));
  backend.EnableVertexAttrib(4);
  backend.DefineVertexAttribFloat(4, 4, sizeof(InstanceGpuData), offsetof(InstanceGpuData, modelRow1));
  backend.EnableVertexAttrib(5);
  backend.DefineVertexAttribFloat(5, 4, sizeof(InstanceGpuData), offsetof(InstanceGpuData, modelRow2));
  backend.EnableVertexAttrib(6);
  backend.DefineVertexAttribFloat(6, 4, sizeof(InstanceGpuData), offsetof(InstanceGpuData, modelRow3));
  backend.SetVertexAttribDivisor(3, 1);
  backend.SetVertexAttribDivisor(4, 1);
  backend.SetVertexAttribDivisor(5, 1);
  backend.SetVertexAttribDivisor(6, 1);

  backend.BindVertexArray(0);

  gpuLod.vertexCount = static_cast<int>(cpuLod.vertices.size());
  gpuLod.indexCount = static_cast<int>(cpuLod.indices.size());
  return true;
}

void CadModelRenderFeature::DestroyMesh(GpuMesh& gpu) {
  for (auto& lod : gpu.lods) {
    lod.instanceVbo.Reset();
    lod.ebo.Reset();
    lod.vbo.Reset();
    lod.vao.Reset();
    lod.vertexCount = 0;
    lod.indexCount = 0;
  }
  gpu.lods.clear();
  if (gpu.shadowProxy.has_value()) {
    auto& lod = gpu.shadowProxy.value();
    lod.instanceVbo.Reset();
    lod.ebo.Reset();
    lod.vbo.Reset();
    lod.vao.Reset();
    lod.vertexCount = 0;
    lod.indexCount = 0;
    gpu.shadowProxy.reset();
  }
  gpu.boundsCenter = glm::vec3{0.0F, 0.0F, 0.0F};
  gpu.boundsRadius = 1.0F;
  gpu.materialHints = PositionNormalMesh::MaterialHints{};
}

std::size_t CadModelRenderFeature::EstimateGpuMeshBytes(const GpuMesh& gpu) {
  std::size_t bytes = 0;
  const auto accumulateLod = [&](const GpuMesh::LodGpu& lod) {
    const std::size_t vertexCount = static_cast<std::size_t>(std::max(lod.vertexCount, 0));
    const std::size_t indexCount = static_cast<std::size_t>(std::max(lod.indexCount, 0));
    bytes += vertexCount * sizeof(cadfeature::PackedVertex);
    bytes += indexCount * sizeof(std::uint32_t);
    bytes += sizeof(cadfeature::InstanceGpuData);
  };
  for (const auto& lod : gpu.lods) {
    accumulateLod(lod);
  }
  if (gpu.shadowProxy.has_value()) {
    accumulateLod(gpu.shadowProxy.value());
  }
  return bytes;
}

std::size_t CadModelRenderFeature::SelectLodLevel(
    const GpuMesh& mesh,
    const glm::mat4& mvp,
    const int viewportWidth,
    const int viewportHeight) {
  if (mesh.lods.size() <= 1) {
    return 0;
  }

  const CadLodPolicy& policy = LoadCadLodPolicy();
  const float radiusPx = ComputeScreenSpaceRadiusPixels(
      mesh.boundsCenter,
      mesh.boundsRadius,
      mvp,
      viewportWidth,
      viewportHeight);

  std::size_t selected = mesh.lods.size() - 1;
  float threshold = policy.lod0ScreenRadiusPx;
  for (std::size_t i = 0; i < mesh.lods.size(); ++i) {
    if (i == mesh.lods.size() - 1 || radiusPx >= threshold) {
      selected = i;
      break;
    }
    threshold *= policy.screenRadiusDecay;
  }

  if (policy.maxPreferredDrawIndices > 0) {
    while (selected + 1 < mesh.lods.size() &&
           static_cast<std::size_t>(std::max(mesh.lods[selected].indexCount, 0)) > policy.maxPreferredDrawIndices) {
      ++selected;
    }
  }
  return selected;
}

}  // namespace repulsor3d
