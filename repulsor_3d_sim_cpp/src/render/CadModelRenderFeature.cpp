#include "repulsor3d/render/CadModelRenderFeature.hpp"

#include <GL/glew.h>

#include <algorithm>
#include <utility>

#include "repulsor3d/Renderer.hpp"
#include "repulsor3d/render/cad/CadBroadphase.hpp"
#include "repulsor3d/render/cad/CadPolicies.hpp"

namespace repulsor3d {

using cad::CadShadowPolicy;
using cad::GetEnvBool;
using cad::GetEnvFloat;
using cad::GetEnvInt;
using cad::LoadCadShadowPolicy;

CadModelRenderFeature::CadModelRenderFeature(
    const RenderPass renderPass,
    std::string featureName,
    std::vector<std::string> dependencies)
    : renderPass_(renderPass), featureName_(std::move(featureName)), dependencies_(std::move(dependencies)) {}

void CadModelRenderFeature::FlatLitMaterialPass::Apply(const MaterialInstance& material) {
  if (colorUniformLocation_ >= 0) {
    const auto& color = material.definition.baseColor;
    backend_.SetUniformVec4(colorUniformLocation_, color.r, color.g, color.b, color.a);
  }
}

void CadModelRenderFeature::WireframeMaterialPass::Apply(const MaterialInstance& material) {
  backend_.SetWireframeMode(material.definition.wireframe);
}

CadModelRenderFeature::~CadModelRenderFeature() {
  StopLoadWorker();
  for (auto& [_, pending] : pendingLoads_) {
    if (pending.status != nullptr) {
      pending.status->cancelRequested = true;
    }
    if (pending.future.valid()) {
      try {
        pending.future.wait();
      } catch (...) {
      }
    }
  }
  pendingLoads_.clear();
  for (auto& [_, pendingUpload] : pendingGpuUploads_) {
    DestroyMesh(pendingUpload.gpu);
  }
  pendingGpuUploads_.clear();

  for (auto& [_, mesh] : meshCache_) {
    DestroyMesh(mesh);
  }
  meshCache_.clear();
  textureCache_.clear();
  failedTextureLoads_.clear();

  shadowDepthTexture_.Reset();
  shadowDepthTextureFar_.Reset();
  if (shadowFbo_ != 0) {
    glDeleteFramebuffers(1, &shadowFbo_);
    shadowFbo_ = 0;
  }
  if (shadowFboFar_ != 0) {
    glDeleteFramebuffers(1, &shadowFboFar_);
    shadowFboFar_ = 0;
  }
  shadowShader_.Reset();
  shader_.Reset();
}

bool CadModelRenderFeature::Initialize(Renderer& renderer) {
  renderer_ = &renderer;
  geometryProvider_ = &renderer.GetGeometryProvider();
  meshProvider_ = CreateDefaultMeshProvider(*geometryProvider_);
  backend_ = &renderer.GetRenderBackend();
  uploadBudgetPerFrame_ = std::max(1, GetEnvInt("CAD_UPLOADS_PER_FRAME", uploadBudgetPerFrame_));
  const CadShadowPolicy& shadowPolicy = LoadCadShadowPolicy();
  shadowEnabled_ = shadowPolicy.enabled;
  shadowMapSize_ = shadowPolicy.mapSize;
  shadowStrength_ = shadowPolicy.strength;
  shadowPcfRadius_ = shadowPolicy.pcfRadius;
  shadowCascadeCount_ = std::clamp(shadowPolicy.cascadeCount, 1, 2);
  shadowCascadeSplitM_ = shadowPolicy.cascadeSplitDistanceM;
  triplanarScale_ = std::max(1e-4F, GetEnvFloat("CAD_TRIPLANAR_SCALE", triplanarScale_));
  autoQualityEnabled_ = GetEnvBool("CAD_PERF_AUTO_QUALITY", autoQualityEnabled_);
  perfTargetFrameMs_ = std::clamp(GetEnvFloat("CAD_PERF_TARGET_FRAME_MS", perfTargetFrameMs_), 8.0F, 66.0F);
  maxAutoLodBias_ = std::clamp(GetEnvInt("CAD_AUTO_MAX_LOD_BIAS", maxAutoLodBias_), 0, 3);
  shadowLodBias_ = std::clamp(GetEnvInt("CAD_SHADOW_LOD_BIAS", shadowLodBias_), 0, 3);
  shadowCasterMaxIndices_ = std::clamp(
      GetEnvInt("CAD_SHADOW_CAST_MAX_INDICES", shadowCasterMaxIndices_),
      0,
      50000000);
  fastShadingMinIndices_ = std::clamp(
      GetEnvInt("CAD_FAST_SHADING_MIN_INDICES", fastShadingMinIndices_),
      0,
      50000000);
  runtimeMaxDrawIndices_ = std::clamp(
      GetEnvInt("CAD_RUNTIME_MAX_DRAW_INDICES", runtimeMaxDrawIndices_),
      0,
      50000000);
  runtimeMaxDrawIndicesField_ = std::clamp(
      GetEnvInt("CAD_RUNTIME_MAX_DRAW_INDICES_FIELD", runtimeMaxDrawIndicesField_),
      0,
      50000000);
  StartLoadWorker();
  if (initialized_) {
    return true;
  }

  initialized_ = CreateShader();
  return initialized_;
}

void CadModelRenderFeature::StartLoadWorker() {
  loadScheduler_.Start(1);
}

void CadModelRenderFeature::StopLoadWorker() {
  loadScheduler_.Stop();
}

void CadModelRenderFeature::EnqueueLoadTask(std::packaged_task<PreparedCpuMesh()>&& task) {
  loadScheduler_.Enqueue(std::move(task));
}

}  // namespace repulsor3d
