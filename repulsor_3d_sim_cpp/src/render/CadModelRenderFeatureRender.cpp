#include "repulsor3d/render/CadModelRenderFeature.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <variant>
#include <vector>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/trigonometric.hpp>

#include "repulsor3d/render/cad/CadBroadphase.hpp"
#include "repulsor3d/render/cad/CadFrustum.hpp"
#include "repulsor3d/render/cad/CadPolicies.hpp"
#include "cad/feature/CadModelRenderFeatureInternals.hpp"

namespace repulsor3d {

namespace {

std::uint32_t FloatBits(const float value) {
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

}  // namespace

using cad::CadVisualPolicy;
using cad::GetEnvBool;
using cad::LoadCadVisualPolicy;
using cadfeature::BuildBroadphaseFingerprint;
using cadfeature::InstanceGpuData;
using cadfeature::ToInstanceData;

void CadModelRenderFeature::Render(const RenderFeatureContext& context, const RendererDrawApi& /*drawApi*/) {
  if (!initialized_ || shader_.Get() == 0 || geometryProvider_ == nullptr || backend_ == nullptr) {
    return;
  }
  uploadsThisFrame_ = 0;

  if (GetEnvBool("CAD_CANCEL_PENDING_LOADS", false)) {
    for (auto& [_, pending] : pendingLoads_) {
      if (pending.status != nullptr) {
        pending.status->cancelRequested = true;
      }
      pending.cancellationToken.Cancel();
    }
  }

  if (context.diagnosticsWriter != nullptr) {
    float progressSum = 0.0F;
    int progressCount = 0;
    int failedCount = static_cast<int>(failedLoads_.size());
    for (const auto& [_, pending] : pendingLoads_) {
      if (pending.status != nullptr) {
        progressSum += pending.status->progress.load();
        ++progressCount;
        if (pending.status->failed.load()) {
          ++failedCount;
        }
      }
    }
    const float avgProgress = progressCount > 0 ? (progressSum / static_cast<float>(progressCount)) : 0.0F;
    context.diagnosticsWriter->RecordCounter("cad.loads.active", static_cast<double>(pendingLoads_.size()));
    context.diagnosticsWriter->RecordCounter("cad.loads.failed", static_cast<double>(failedCount));
    context.diagnosticsWriter->RecordCounter("cad.loads.progress_pct", static_cast<double>(avgProgress * 100.0F));
    context.diagnosticsWriter->RecordCounter("cad.uploads.pending", static_cast<double>(pendingGpuUploads_.size()));
    context.diagnosticsWriter->RecordCounter(
        "cad.cache.mesh_count",
        static_cast<double>(cacheLifetimeManager_.Stats(ResourceClass::Mesh).count));
    context.diagnosticsWriter->RecordCounter(
        "cad.cache.mesh_bytes",
        static_cast<double>(cacheLifetimeManager_.Stats(ResourceClass::Mesh).bytes));
    context.diagnosticsWriter->RecordCounter(
        "cad.cache.texture_count",
        static_cast<double>(cacheLifetimeManager_.Stats(ResourceClass::Texture).count));
    context.diagnosticsWriter->RecordCounter(
        "cad.cache.texture_bytes",
        static_cast<double>(cacheLifetimeManager_.Stats(ResourceClass::Texture).bytes));
    context.diagnosticsWriter->RecordCounter(
        "cad.cache.over_budget",
        (cacheLifetimeManager_.IsOverBudget(ResourceClass::Mesh) ||
         cacheLifetimeManager_.IsOverBudget(ResourceClass::Texture))
            ? 1.0
            : 0.0);
  }

  if (renderPass_ == RenderPass::Opaque) {
    backend_->SetBlendEnabled(false);
  } else {
    backend_->SetBlendEnabled(true);
  }

  backend_->UseProgram(shader_.Get());
  const CadVisualPolicy& visualPolicy = LoadCadVisualPolicy();
  const auto frustumPlanes = cad::ExtractFrustumPlanes(context.viewProjection);
  backend_->SetUniformVec3(uLightDirLoc_, -0.3F, -0.5F, -1.0F);
  backend_->SetUniformVec3(
      uCameraPosLoc_,
      context.cameraWorldPosition.x,
      context.cameraWorldPosition.y,
      context.cameraWorldPosition.z);
  backend_->SetUniform1f(uDepthCueStrengthLoc_, visualPolicy.depthCueStrength);
  backend_->SetUniform1f(uSpecularStrengthLoc_, visualPolicy.specularStrength);
  backend_->SetUniform1f(uRimStrengthLoc_, visualPolicy.rimStrength);
  backend_->SetUniform1f(uKeyLightIntensityLoc_, visualPolicy.keyLightIntensity);
  backend_->SetUniform1f(uFillLightIntensityLoc_, visualPolicy.fillLightIntensity);
  backend_->SetUniform1f(uAmbientStrengthLoc_, visualPolicy.ambientStrength);
  backend_->SetUniform1f(uRoughnessLoc_, visualPolicy.roughness);
  backend_->SetUniform1f(uMetallicLoc_, visualPolicy.metallic);
  backend_->SetUniform1f(uExposureLoc_, visualPolicy.exposure);
  backend_->SetUniform1f(uFogDensityLoc_, visualPolicy.fogDensity);
  backend_->SetUniform1f(uSaturationLoc_, visualPolicy.saturation);
  backend_->SetUniform1f(uGammaLoc_, visualPolicy.gamma);

  int adaptiveLodBias = 0;
  bool reduceShadowQuality = false;
  bool disableShadowsThisFrame = false;
  double frameAverageMs = 0.0;
  if (context.diagnostics != nullptr) {
    frameAverageMs = context.diagnostics->frameAverageMilliseconds > 0.0
                         ? context.diagnostics->frameAverageMilliseconds
                         : context.diagnostics->frameMilliseconds;
  }
  if (autoQualityEnabled_ && frameAverageMs > 0.0) {
    if (frameAverageMs > static_cast<double>(perfTargetFrameMs_ * 1.35F)) {
      adaptiveLodBias = maxAutoLodBias_;
    } else if (frameAverageMs > static_cast<double>(perfTargetFrameMs_ * 1.15F)) {
      adaptiveLodBias = std::min(1, maxAutoLodBias_);
    }
    reduceShadowQuality = frameAverageMs > static_cast<double>(perfTargetFrameMs_ * 1.25F);
    disableShadowsThisFrame = frameAverageMs > static_cast<double>(perfTargetFrameMs_ * 1.60F);
  }
  if (context.diagnosticsWriter != nullptr) {
    context.diagnosticsWriter->RecordCounter("cad.auto.lod_bias", static_cast<double>(adaptiveLodBias));
    context.diagnosticsWriter->RecordCounter("cad.auto.shadow_reduced", reduceShadowQuality ? 1.0 : 0.0);
    context.diagnosticsWriter->RecordCounter("cad.auto.shadow_disabled", disableShadowsThisFrame ? 1.0 : 0.0);
    context.diagnosticsWriter->RecordCounter("cad.auto.frame_ms", frameAverageMs);
  }

  struct DrawItem {
    const GpuMesh* mesh = nullptr;
    const GpuMesh::LodGpu* lod = nullptr;
    const GpuMesh::LodGpu* shadowLod = nullptr;
    const GpuTexture* albedoTexture = nullptr;
    const GpuTexture* normalTexture = nullptr;
    glm::mat4 model{1.0F};
    glm::vec4 color{1.0F};
    float roughness = 0.72F;
    float metallic = 0.0F;
    float normalStrength = 1.0F;
    bool useAssetColor = false;
    bool fastShading = false;
    bool aggressiveLodCap = false;
    bool wireframe = false;
    bool mirroredWinding = false;
  };
  std::vector<DrawItem> drawItems;
  drawItems.reserve(256);
  std::vector<glm::vec3> drawCenters;
  std::vector<float> drawRadii;
  drawCenters.reserve(256);
  drawRadii.reserve(256);

  for (const auto& command : context.commandBuffer) {
    std::visit(
        [&](const auto& typed) {
          using CommandType = std::decay_t<decltype(typed)>;
          if constexpr (!std::is_same_v<CommandType, DrawMeshInstanceCommand>) {
            return;
          } else {
            if (typed.pass != renderPass_) {
              return;
            }

            const auto& instance = typed.primitive;
            const GpuMesh* mesh = GetOrLoadMesh(instance.assetPath);
            if (mesh == nullptr || mesh->lods.empty()) {
              return;
            }

            glm::mat4 model(1.0F);
            model = glm::translate(model, instance.position);
            model = glm::rotate(model, glm::radians(instance.rotationDeg.x), glm::vec3{1.0F, 0.0F, 0.0F});
            model = glm::rotate(model, glm::radians(instance.rotationDeg.y), glm::vec3{0.0F, 1.0F, 0.0F});
            model = glm::rotate(model, glm::radians(instance.rotationDeg.z), glm::vec3{0.0F, 0.0F, 1.0F});
            model = glm::scale(model, instance.scale);
            if (instance.centerOnMeshBounds) {
              model =
                  model * glm::translate(glm::mat4(1.0F), glm::vec3{-mesh->boundsCenter.x, -mesh->boundsCenter.y, 0.0F});
            }

            if (visualPolicy.enableFrustumCulling) {
              const glm::vec3 worldBoundsCenter = glm::vec3(model * glm::vec4(mesh->boundsCenter, 1.0F));
              const glm::vec3 scaleAxisX{model[0][0], model[0][1], model[0][2]};
              const glm::vec3 scaleAxisY{model[1][0], model[1][1], model[1][2]};
              const glm::vec3 scaleAxisZ{model[2][0], model[2][1], model[2][2]};
              const float maxScale =
                  std::max(glm::length(scaleAxisX), std::max(glm::length(scaleAxisY), glm::length(scaleAxisZ)));
              const float worldBoundsRadius = std::max(mesh->boundsRadius * std::max(maxScale, 1e-4F), 1e-4F);
              drawCenters.push_back(worldBoundsCenter);
              drawRadii.push_back(worldBoundsRadius);
            }

            const glm::mat4 mvp = context.viewProjection * model;
            const std::size_t lodIndex = SelectLodLevel(*mesh, mvp, context.viewportWidth, context.viewportHeight);
            std::size_t drawLodIndex = std::min(
                lodIndex + static_cast<std::size_t>(std::max(0, adaptiveLodBias)),
                mesh->lods.size() - 1);
            const int activeRuntimeDrawCap = instance.centerOnMeshBounds
                                                 ? runtimeMaxDrawIndicesField_
                                                 : runtimeMaxDrawIndices_;
            if (activeRuntimeDrawCap > 0) {
              while (drawLodIndex + 1 < mesh->lods.size() &&
                     std::max(mesh->lods[drawLodIndex].indexCount, 0) > activeRuntimeDrawCap) {
                ++drawLodIndex;
              }
            }
            std::size_t shadowLodIndex = std::min(
                drawLodIndex + static_cast<std::size_t>(std::max(0, shadowLodBias_)),
                mesh->lods.size() - 1);
            if (reduceShadowQuality) {
              shadowLodIndex = std::min(shadowLodIndex + 1, mesh->lods.size() - 1);
            }
            const auto& lod = mesh->lods[drawLodIndex];
            const auto& shadowLod = mesh->lods[shadowLodIndex];
            const GpuMesh::LodGpu* selectedShadowLod =
                mesh->shadowProxy.has_value() ? &mesh->shadowProxy.value() : &shadowLod;
            const int drawIndexCount = std::max(lod.indexCount, 0);
            const bool fastShading =
                fastShadingMinIndices_ > 0 && drawIndexCount > fastShadingMinIndices_;

            const std::string albedoPath = !instance.albedoTexturePath.empty()
                                               ? instance.albedoTexturePath
                                               : mesh->materialHints.baseColorTexturePath;
            const std::string normalPath = !instance.normalTexturePath.empty()
                                               ? instance.normalTexturePath
                                               : mesh->materialHints.normalTexturePath;
            const GpuTexture* albedoTexture = GetOrLoadTexture(albedoPath);
            const GpuTexture* normalTexture = GetOrLoadTexture(normalPath);

            drawItems.push_back(
                {.mesh = mesh,
                 .lod = &lod,
                 .shadowLod = selectedShadowLod,
                 .albedoTexture = albedoTexture,
                 .normalTexture = normalTexture,
                 .model = model,
                 .color = instance.color,
                 .roughness =
                     (instance.roughnessOverride >= 0.0F
                          ? instance.roughnessOverride
                          : (mesh->materialHints.roughness >= 0.0F ? mesh->materialHints.roughness : visualPolicy.roughness)),
                 .metallic =
                     (instance.metallicOverride >= 0.0F
                          ? instance.metallicOverride
                          : (mesh->materialHints.metallic >= 0.0F ? mesh->materialHints.metallic : visualPolicy.metallic)),
                 .normalStrength = instance.normalStrength,
                 .useAssetColor = instance.useAssetColor,
                 .fastShading = fastShading,
                 .aggressiveLodCap = instance.centerOnMeshBounds,
                 .wireframe = instance.wireframe,
                 .mirroredWinding = glm::determinant(glm::mat3(model)) < 0.0F});
          }
        },
        command);
  }

  if (visualPolicy.enableFrustumCulling && !drawItems.empty()) {
    std::vector<DrawItem> filtered;
    filtered.reserve(drawItems.size());
    if (visualPolicy.enableBroadphase) {
      cad::Aabb frustumAabb;
      if (cad::TryExtractFrustumAabb(context.viewProjection, frustumAabb)) {
        const float broadphaseCellSize = std::max(0.1F, visualPolicy.broadphaseCellSizeM);
        if (broadphaseCache_ == nullptr ||
            std::abs(broadphaseCacheCellSizeM_ - broadphaseCellSize) > 1e-5F) {
          broadphaseCache_ = std::make_unique<cad::UniformGridBroadphase>(broadphaseCellSize);
          broadphaseCacheCellSizeM_ = broadphaseCellSize;
          broadphaseCacheFingerprint_ = 0ULL;
        }

        const std::uint64_t broadphaseFingerprint =
            BuildBroadphaseFingerprint(drawCenters, drawRadii, broadphaseCellSize);
        if (broadphaseFingerprint != broadphaseCacheFingerprint_) {
          broadphaseCache_->Clear();
          for (std::size_t i = 0; i < drawItems.size(); ++i) {
            broadphaseCache_->Insert(cad::BoundingSphereRef{drawCenters[i], drawRadii[i], i});
          }
          broadphaseCacheFingerprint_ = broadphaseFingerprint;
        }
        std::vector<std::size_t> candidates;
        broadphaseCache_->QueryAabb(frustumAabb, candidates);
        for (const std::size_t index : candidates) {
          if (index >= drawItems.size()) {
            continue;
          }
          if (cad::IsSphereVisible(frustumPlanes, drawCenters[index], drawRadii[index])) {
            filtered.push_back(drawItems[index]);
          }
        }
      } else {
        for (std::size_t i = 0; i < drawItems.size(); ++i) {
          if (cad::IsSphereVisible(frustumPlanes, drawCenters[i], drawRadii[i])) {
            filtered.push_back(drawItems[i]);
          }
        }
      }
    } else {
      for (std::size_t i = 0; i < drawItems.size(); ++i) {
        if (cad::IsSphereVisible(frustumPlanes, drawCenters[i], drawRadii[i])) {
          filtered.push_back(drawItems[i]);
        }
      }
    }
    drawItems.swap(filtered);
  }

  if (drawItems.empty()) {
    backend_->SetBlendEnabled(true);
    backend_->SetWireframeMode(false);
    return;
  }
  if (context.diagnosticsWriter != nullptr) {
    double totalIndices = 0.0;
    for (const auto& item : drawItems) {
      if (item.lod == nullptr) {
        continue;
      }
      const int indexCount = std::max(item.lod->indexCount, 0);
      totalIndices += static_cast<double>(indexCount > 0 ? indexCount : std::max(item.lod->vertexCount, 0));
    }
    context.diagnosticsWriter->RecordCounter("cad.draw.items", static_cast<double>(drawItems.size()));
    context.diagnosticsWriter->RecordCounter("cad.draw.indices", totalIndices);
  }

  glm::vec3 sceneCenter{0.0F, 0.0F, 0.0F};
  float sceneRadius = 1.0F;
  {
    glm::vec3 minP{std::numeric_limits<float>::max()};
    glm::vec3 maxP{std::numeric_limits<float>::lowest()};
    for (const auto& item : drawItems) {
      const glm::vec3 c = glm::vec3(item.model * glm::vec4(item.mesh->boundsCenter, 1.0F));
      const glm::vec3 scaleAxisX{item.model[0][0], item.model[0][1], item.model[0][2]};
      const glm::vec3 scaleAxisY{item.model[1][0], item.model[1][1], item.model[1][2]};
      const glm::vec3 scaleAxisZ{item.model[2][0], item.model[2][1], item.model[2][2]};
      const float maxScale =
          std::max(glm::length(scaleAxisX), std::max(glm::length(scaleAxisY), glm::length(scaleAxisZ)));
      const float r = std::max(item.mesh->boundsRadius * std::max(maxScale, 1e-4F), 0.1F);
      minP = glm::min(minP, c - glm::vec3(r, r, r));
      maxP = glm::max(maxP, c + glm::vec3(r, r, r));
    }
    sceneCenter = 0.5F * (minP + maxP);
    sceneRadius = std::max(glm::length(maxP - minP) * 0.6F, 1.0F);
  }

  const glm::vec3 lightDir = glm::normalize(glm::vec3{-0.3F, -0.5F, -1.0F});
  const float nearCascadeRadius = std::clamp(
      shadowCascadeSplitM_ * 0.8F,
      std::max(2.0F, sceneRadius * 0.25F),
      sceneRadius);
  const glm::vec3 lightEyeNear = sceneCenter - lightDir * (nearCascadeRadius * 2.3F);
  const glm::mat4 lightViewNear = glm::lookAt(lightEyeNear, sceneCenter, glm::vec3{0.0F, 0.0F, 1.0F});
  const glm::mat4 lightProjNear = glm::ortho(
      -nearCascadeRadius,
      nearCascadeRadius,
      -nearCascadeRadius,
      nearCascadeRadius,
      0.1F,
      nearCascadeRadius * 5.0F + 0.1F);
  const glm::mat4 lightViewProjection = lightProjNear * lightViewNear;

  glm::mat4 lightViewProjectionFar = lightViewProjection;
  const bool farCascadeRequested = shadowCascadeCount_ > 1 && !reduceShadowQuality;
  if (farCascadeRequested) {
    const glm::vec3 lightEyeFar = sceneCenter - lightDir * (sceneRadius * 2.3F);
    const glm::mat4 lightViewFar = glm::lookAt(lightEyeFar, sceneCenter, glm::vec3{0.0F, 0.0F, 1.0F});
    const glm::mat4 lightProjFar = glm::ortho(
        -sceneRadius,
        sceneRadius,
        -sceneRadius,
        sceneRadius,
        0.1F,
        sceneRadius * 5.0F + 0.1F);
    lightViewProjectionFar = lightProjFar * lightViewFar;
  }

  const bool nearShadowReady =
      shadowEnabled_ && !disableShadowsThisFrame && shadowFbo_ != 0 && shadowShader_.Get() != 0;
  const bool farShadowReady = nearShadowReady && farCascadeRequested && shadowFboFar_ != 0;
  bool shadowsEnabledForShading = false;

  if (nearShadowReady) {
    GLint oldViewport[4] = {0, 0, 0, 0};
    glGetIntegerv(GL_VIEWPORT, oldViewport);

    struct ShadowBatch {
      const GpuMesh::LodGpu* lod = nullptr;
      bool mirroredWinding = false;
      std::vector<InstanceGpuData> instances;
    };
    std::vector<ShadowBatch> shadowBatches;
    shadowBatches.reserve(drawItems.size());
    int shadowCasterSkipped = 0;
    for (const auto& item : drawItems) {
      const int shadowIndexCount = (item.shadowLod != nullptr) ? std::max(item.shadowLod->indexCount, 0) : 0;
      if (shadowCasterMaxIndices_ > 0 && shadowIndexCount > shadowCasterMaxIndices_) {
        ++shadowCasterSkipped;
        continue;
      }
      auto it = std::find_if(shadowBatches.begin(), shadowBatches.end(), [&](const ShadowBatch& batch) {
        return batch.lod == item.shadowLod && batch.mirroredWinding == item.mirroredWinding;
      });
      if (it == shadowBatches.end()) {
        ShadowBatch batch;
        batch.lod = item.shadowLod;
        batch.mirroredWinding = item.mirroredWinding;
        batch.instances.push_back(ToInstanceData(item.model));
        shadowBatches.push_back(std::move(batch));
      } else {
        it->instances.push_back(ToInstanceData(item.model));
      }
    }
    if (context.diagnosticsWriter != nullptr) {
      double shadowIndices = 0.0;
      for (const auto& batch : shadowBatches) {
        if (batch.lod == nullptr) {
          continue;
        }
        const int idxCount = std::max(batch.lod->indexCount, 0);
        shadowIndices += static_cast<double>(idxCount > 0 ? idxCount : std::max(batch.lod->vertexCount, 0)) *
                         static_cast<double>(batch.instances.size());
      }
      context.diagnosticsWriter->RecordCounter("cad.shadow.casters_skipped", static_cast<double>(shadowCasterSkipped));
      context.diagnosticsWriter->RecordCounter("cad.shadow.caster_batches", static_cast<double>(shadowBatches.size()));
      context.diagnosticsWriter->RecordCounter("cad.shadow.indices", shadowIndices);
    }

    auto renderShadowPass = [&](const unsigned int fbo, const glm::mat4& lightViewProj) {
      if (fbo == 0) {
        return;
      }
      glBindFramebuffer(GL_FRAMEBUFFER, fbo);
      glViewport(0, 0, shadowMapSize_, shadowMapSize_);
      glClear(GL_DEPTH_BUFFER_BIT);
      glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
      glCullFace(GL_FRONT);
      backend_->UseProgram(shadowShader_.Get());
      backend_->SetUniformMat4(uShadowLightMvpLoc_, &lightViewProj[0][0]);

      for (const auto& batch : shadowBatches) {
        if (batch.lod == nullptr || batch.instances.empty()) {
          continue;
        }
        if (batch.mirroredWinding) {
          glFrontFace(GL_CW);
        }
        backend_->BindVertexArray(batch.lod->vao.Get());
        backend_->BindArrayBuffer(batch.lod->instanceVbo.Get());
        backend_->UploadArrayBufferData(batch.instances.size() * sizeof(InstanceGpuData), batch.instances.data(), true);
        if (batch.lod->indexCount > 0) {
          backend_->DrawIndexedTrianglesInstanced(batch.lod->indexCount, static_cast<int>(batch.instances.size()));
        } else {
          backend_->DrawTrianglesInstanced(batch.lod->vertexCount, static_cast<int>(batch.instances.size()));
        }
        backend_->BindVertexArray(0);
        if (batch.mirroredWinding) {
          glFrontFace(GL_CCW);
        }
      }
    };

    const bool hasShadowCasters = !shadowBatches.empty();
    shadowsEnabledForShading = hasShadowCasters;
    if (hasShadowCasters) {
      renderShadowPass(shadowFbo_, lightViewProjection);
      if (farShadowReady) {
        renderShadowPass(shadowFboFar_, lightViewProjectionFar);
      }
    } else {
      glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
      glViewport(0, 0, shadowMapSize_, shadowMapSize_);
      glClear(GL_DEPTH_BUFFER_BIT);
      if (farShadowReady) {
        glBindFramebuffer(GL_FRAMEBUFFER, shadowFboFar_);
        glViewport(0, 0, shadowMapSize_, shadowMapSize_);
        glClear(GL_DEPTH_BUFFER_BIT);
      }
    }

    glCullFace(GL_BACK);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(oldViewport[0], oldViewport[1], oldViewport[2], oldViewport[3]);
  }

  backend_->UseProgram(shader_.Get());
  const glm::mat4& lightViewProjectionFarUniform = farShadowReady ? lightViewProjectionFar : lightViewProjection;
  backend_->SetUniformMat4(uMvpLoc_, &context.viewProjection[0][0]);
  backend_->SetUniformMat4(uLightViewProjectionLoc_, &lightViewProjection[0][0]);
  backend_->SetUniformMat4(uLightViewProjectionFarLoc_, &lightViewProjectionFarUniform[0][0]);
  backend_->SetUniform1i(uShadowEnabledLoc_, shadowsEnabledForShading ? 1 : 0);
  backend_->SetUniform1f(uShadowStrengthLoc_, shadowStrength_);
  backend_->SetUniform1i(uShadowPcfRadiusLoc_, reduceShadowQuality ? std::max(1, shadowPcfRadius_ - 1) : shadowPcfRadius_);
  backend_->SetUniform1i(uShadowCascadeCountLoc_, farShadowReady ? 2 : 1);
  backend_->SetUniform1f(uShadowCascadeSplitMLoc_, shadowCascadeSplitM_);
  backend_->SetActiveTextureUnit(3);
  backend_->BindTexture2D(shadowDepthTexture_.Get());
  backend_->SetUniform1i(uShadowMapLoc_, 3);
  backend_->SetActiveTextureUnit(4);
  backend_->BindTexture2D(farShadowReady ? shadowDepthTextureFar_.Get() : shadowDepthTexture_.Get());
  backend_->SetUniform1i(uShadowMapFarLoc_, 4);

  struct BatchKey {
    const GpuMesh::LodGpu* lod = nullptr;
    const GpuTexture* albedoTexture = nullptr;
    const GpuTexture* normalTexture = nullptr;
    std::uint32_t colorR = 0;
    std::uint32_t colorG = 0;
    std::uint32_t colorB = 0;
    std::uint32_t colorA = 0;
    std::uint32_t roughness = 0;
    std::uint32_t metallic = 0;
    std::uint32_t normalStrength = 0;
    bool useAssetColor = false;
    bool fastShading = false;
    bool wireframe = false;
    bool mirroredWinding = false;
    bool operator==(const BatchKey& other) const {
      return lod == other.lod &&
             albedoTexture == other.albedoTexture &&
             normalTexture == other.normalTexture &&
             colorR == other.colorR && colorG == other.colorG && colorB == other.colorB && colorA == other.colorA &&
             roughness == other.roughness && metallic == other.metallic &&
             normalStrength == other.normalStrength &&
             useAssetColor == other.useAssetColor && fastShading == other.fastShading &&
             wireframe == other.wireframe &&
             mirroredWinding == other.mirroredWinding;
    }
  };
  struct BatchKeyHash {
    std::size_t operator()(const BatchKey& key) const {
      std::size_t h = reinterpret_cast<std::size_t>(key.lod);
      h ^= reinterpret_cast<std::size_t>(key.albedoTexture) * 2654435761U;
      h ^= reinterpret_cast<std::size_t>(key.normalTexture) * 2246822519U;
      h ^= static_cast<std::size_t>(key.colorR) * 16777619U;
      h ^= static_cast<std::size_t>(key.colorG) * 2166136261U;
      h ^= static_cast<std::size_t>(key.colorB) * 709607U;
      h ^= static_cast<std::size_t>(key.colorA) * 1469598103U;
      h ^= static_cast<std::size_t>(key.roughness) * 1099511628211ULL;
      h ^= static_cast<std::size_t>(key.metallic) * 1315423911U;
      h ^= static_cast<std::size_t>(key.normalStrength) * 374761393U;
      h ^= key.useAssetColor ? 0x1U : 0x0U;
      h ^= key.fastShading ? 0x2U : 0x0U;
      h ^= key.wireframe ? 0x4U : 0x0U;
      h ^= key.mirroredWinding ? 0x8U : 0x0U;
      return h;
    }
  };

  struct DrawBatch {
    glm::vec4 color{1.0F};
    float roughness = 0.72F;
    float metallic = 0.0F;
    float normalStrength = 1.0F;
    const GpuTexture* albedoTexture = nullptr;
    const GpuTexture* normalTexture = nullptr;
    bool useAssetColor = false;
    bool fastShading = false;
    bool wireframe = false;
    bool mirroredWinding = false;
    const GpuMesh::LodGpu* lod = nullptr;
    std::vector<InstanceGpuData> instances;
  };

  std::unordered_map<BatchKey, DrawBatch, BatchKeyHash> batches;
  batches.reserve(drawItems.size());
  for (const auto& item : drawItems) {
    BatchKey key{
        .lod = item.lod,
        .albedoTexture = item.albedoTexture,
        .normalTexture = item.normalTexture,
        .colorR = FloatBits(item.color.r),
        .colorG = FloatBits(item.color.g),
        .colorB = FloatBits(item.color.b),
        .colorA = FloatBits(item.color.a),
        .roughness = FloatBits(item.roughness),
        .metallic = FloatBits(item.metallic),
        .normalStrength = FloatBits(item.normalStrength),
        .useAssetColor = item.useAssetColor,
        .fastShading = item.fastShading,
        .wireframe = item.wireframe,
        .mirroredWinding = item.mirroredWinding,
    };
    auto& batch = batches[key];
    batch.color = item.color;
    batch.roughness = item.roughness;
    batch.metallic = item.metallic;
    batch.normalStrength = item.normalStrength;
    batch.albedoTexture = item.albedoTexture;
    batch.normalTexture = item.normalTexture;
    batch.useAssetColor = item.useAssetColor;
    batch.fastShading = item.fastShading;
    batch.wireframe = item.wireframe;
    batch.mirroredWinding = item.mirroredWinding;
    batch.lod = item.lod;
    batch.instances.push_back(ToInstanceData(item.model));
  }

  for (auto& [_, batch] : batches) {
    if (batch.lod == nullptr || batch.instances.empty()) {
      continue;
    }
    MaterialInstance material;
    material.definition.baseColor = batch.color;
    material.definition.wireframe = batch.wireframe;
    materialPipeline_.Apply(material);
    backend_->SetUniform1f(uRoughnessLoc_, batch.roughness);
    backend_->SetUniform1f(uMetallicLoc_, batch.metallic);
    backend_->SetUniform1f(uNormalStrengthLoc_, batch.normalStrength);
    backend_->SetUniform1f(uTriplanarScaleLoc_, triplanarScale_);
    backend_->SetUniform1f(uUseAssetColorLoc_, batch.useAssetColor ? 1.0F : 0.0F);
    backend_->SetUniform1i(uShadingModeLoc_, batch.fastShading ? 1 : 0);
    const bool useNormalMap = (batch.normalTexture != nullptr) && !batch.fastShading;
    backend_->SetUniform1i(uHasAlbedoMapLoc_, batch.albedoTexture != nullptr ? 1 : 0);
    backend_->SetUniform1i(uHasNormalMapLoc_, useNormalMap ? 1 : 0);
    backend_->SetActiveTextureUnit(5);
    backend_->BindTexture2D(batch.albedoTexture != nullptr ? batch.albedoTexture->texture.Get() : 0);
    backend_->SetUniform1i(uAlbedoMapLoc_, 5);
    backend_->SetActiveTextureUnit(6);
    backend_->BindTexture2D(useNormalMap ? batch.normalTexture->texture.Get() : 0);
    backend_->SetUniform1i(uNormalMapLoc_, 6);

    if (batch.mirroredWinding) {
      glFrontFace(GL_CW);
    }
    backend_->BindVertexArray(batch.lod->vao.Get());
    backend_->BindArrayBuffer(batch.lod->instanceVbo.Get());
    backend_->UploadArrayBufferData(batch.instances.size() * sizeof(InstanceGpuData), batch.instances.data(), true);
    if (batch.lod->indexCount > 0) {
      backend_->DrawIndexedTrianglesInstanced(batch.lod->indexCount, static_cast<int>(batch.instances.size()));
    } else {
      backend_->DrawTrianglesInstanced(batch.lod->vertexCount, static_cast<int>(batch.instances.size()));
    }
    backend_->BindVertexArray(0);
    if (batch.mirroredWinding) {
      glFrontFace(GL_CCW);
    }
  }

  backend_->SetBlendEnabled(true);
  backend_->SetWireframeMode(false);
  backend_->SetActiveTextureUnit(6);
  backend_->BindTexture2D(0);
  backend_->SetActiveTextureUnit(5);
  backend_->BindTexture2D(0);
  backend_->SetActiveTextureUnit(4);
  backend_->BindTexture2D(0);
  backend_->SetActiveTextureUnit(3);
  backend_->BindTexture2D(0);
}

}  // namespace repulsor3d
