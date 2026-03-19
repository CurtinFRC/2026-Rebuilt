
#include "repulsor3d/render/CadModelRenderFeature.hpp"

#include <GL/glew.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <tuple>
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
#include "repulsor3d/render/cad/CadVisibility.hpp"
#include "cad/feature/CadModelRenderFeatureInternals.hpp"

namespace repulsor3d {

namespace {

std::uint32_t FloatBits(const float value) {
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

constexpr std::uint64_t kFnvOffsetBasis = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

void HashMixU64(std::uint64_t& hash, const std::uint64_t value) {
  std::uint64_t v = value;
  for (int i = 0; i < 8; ++i) {
    hash ^= static_cast<std::uint8_t>(v & 0xFFULL);
    hash *= kFnvPrime;
    v >>= 8U;
  }
}

void HashMixFloat(std::uint64_t& hash, const float value) {
  HashMixU64(hash, static_cast<std::uint64_t>(FloatBits(value)));
}

struct DrawElementsIndirectCommand {
  std::uint32_t count = 0;
  std::uint32_t instanceCount = 1;
  std::uint32_t firstIndex = 0;
  std::uint32_t baseVertex = 0;
  std::uint32_t baseInstance = 0;
};

float ComputeDepth01(const glm::mat4& viewProjection, const glm::vec3& worldPoint) {
  const glm::vec4 clip = viewProjection * glm::vec4(worldPoint, 1.0F);
  if (std::abs(clip.w) <= 1e-6F) {
    return 1.0F;
  }
  return std::clamp((clip.z / clip.w) * 0.5F + 0.5F, 0.0F, 1.0F);
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
  static bool mdiRuntimeDisabled = false;
  const bool allowMdi = !mdiRuntimeDisabled &&
                        backend_->Capabilities().supportsMultiDrawIndirect &&
                        GetEnvBool("CAD_MULTI_DRAW_INDIRECT", true);
  const std::uint32_t clusterMergeGap =
      static_cast<std::uint32_t>(std::max(0, visualPolicy.clusterMergeGapIndices));
  if (allowMdi && cadIndirectDrawBuffer_.Get() == 0) {
    cadIndirectDrawBuffer_.Set(backend_->CreateBuffer());
  }
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
    glm::vec3 worldBoundsCenter{0.0F, 0.0F, 0.0F};
    float worldBoundsRadius = 1.0F;
    float worldMaxScale = 1.0F;
    float depth01 = 1.0F;
    bool useAssetColor = false;
    bool fastShading = false;
    bool aggressiveLodCap = false;
    bool wireframe = false;
    bool mirroredWinding = false;
  };
  std::vector<DrawItem> drawItems;
  drawItems.reserve(256);

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

            const glm::vec3 scaleAxisX{model[0][0], model[0][1], model[0][2]};
            const glm::vec3 scaleAxisY{model[1][0], model[1][1], model[1][2]};
            const glm::vec3 scaleAxisZ{model[2][0], model[2][1], model[2][2]};
            const float maxScale =
                std::max(glm::length(scaleAxisX), std::max(glm::length(scaleAxisY), glm::length(scaleAxisZ)));
            const glm::vec3 worldBoundsCenter = glm::vec3(model * glm::vec4(mesh->boundsCenter, 1.0F));
            const float worldBoundsRadius = std::max(mesh->boundsRadius * std::max(maxScale, 1e-4F), 1e-4F);

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
                 .worldBoundsCenter = worldBoundsCenter,
                 .worldBoundsRadius = worldBoundsRadius,
                 .worldMaxScale = maxScale,
                 .depth01 = ComputeDepth01(context.viewProjection, worldBoundsCenter),
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
    const float frustumCullMarginM = std::max(0.0F, visualPolicy.frustumCullMarginM);
    std::vector<DrawItem> filtered;
    filtered.reserve(drawItems.size());
    if (visualPolicy.enableBroadphase) {
      cad::Aabb frustumAabb;
      if (cad::TryExtractFrustumAabb(context.viewProjection, frustumAabb)) {
        const float broadphaseCellSize = std::max(0.1F, visualPolicy.broadphaseCellSizeM);
        const float broadphasePadding = std::max(frustumCullMarginM, broadphaseCellSize * 0.35F);
        frustumAabb.min -= glm::vec3{broadphasePadding, broadphasePadding, broadphasePadding};
        frustumAabb.max += glm::vec3{broadphasePadding, broadphasePadding, broadphasePadding};
        if (broadphaseCache_ == nullptr ||
            std::abs(broadphaseCacheCellSizeM_ - broadphaseCellSize) > 1e-5F) {
          broadphaseCache_ = std::make_unique<cad::UniformGridBroadphase>(broadphaseCellSize);
          broadphaseCacheCellSizeM_ = broadphaseCellSize;
          broadphaseCacheFingerprint_ = 0ULL;
        }

        std::vector<glm::vec3> centers;
        std::vector<float> radii;
        centers.reserve(drawItems.size());
        radii.reserve(drawItems.size());
        for (const auto& item : drawItems) {
          centers.push_back(item.worldBoundsCenter);
          radii.push_back(item.worldBoundsRadius);
        }

        const std::uint64_t broadphaseFingerprint =
            BuildBroadphaseFingerprint(centers, radii, broadphaseCellSize);
        if (broadphaseFingerprint != broadphaseCacheFingerprint_) {
          broadphaseCache_->Clear();
          for (std::size_t i = 0; i < drawItems.size(); ++i) {
            broadphaseCache_->Insert(cad::BoundingSphereRef{drawItems[i].worldBoundsCenter, drawItems[i].worldBoundsRadius, i});
          }
          broadphaseCacheFingerprint_ = broadphaseFingerprint;
        }
        std::vector<std::size_t> candidates;
        broadphaseCache_->QueryAabb(frustumAabb, candidates);
        for (const std::size_t index : candidates) {
          if (index >= drawItems.size()) {
            continue;
          }
          if (cad::IsSphereVisible(
                  frustumPlanes,
                  drawItems[index].worldBoundsCenter,
                  drawItems[index].worldBoundsRadius,
                  frustumCullMarginM)) {
            filtered.push_back(drawItems[index]);
          }
        }
      } else {
        for (const auto& item : drawItems) {
          if (cad::IsSphereVisible(frustumPlanes, item.worldBoundsCenter, item.worldBoundsRadius, frustumCullMarginM)) {
            filtered.push_back(item);
          }
        }
      }
    } else {
      for (const auto& item : drawItems) {
        if (cad::IsSphereVisible(frustumPlanes, item.worldBoundsCenter, item.worldBoundsRadius, frustumCullMarginM)) {
          filtered.push_back(item);
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

  std::sort(drawItems.begin(), drawItems.end(), [](const DrawItem& a, const DrawItem& b) {
    return a.depth01 < b.depth01;
  });

  cad::TiledOcclusionBuffer occlusionBuffer;
  if (visualPolicy.enableOcclusionCulling) {
    occlusionBuffer.Reset(
        context.viewportWidth,
        context.viewportHeight,
        std::max(8, visualPolicy.hzbTilesX),
        std::max(8, visualPolicy.hzbTilesY));
  }

  struct DrawRange {
    std::uint32_t firstIndex = 0;
    std::uint32_t indexCount = 0;
    float depth01 = 1.0F;
  };
  struct DrawCall {
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
    float depth01 = 1.0F;
    bool useAssetColor = false;
    bool fastShading = false;
    bool wireframe = false;
    bool mirroredWinding = false;
    std::vector<DrawRange> ranges;
  };

  std::vector<DrawCall> drawCalls;
  drawCalls.reserve(drawItems.size());
  std::size_t totalClusters = 0;
  std::size_t culledClusters = 0;
  for (const auto& item : drawItems) {
    if (item.lod == nullptr) {
      continue;
    }

    std::vector<DrawRange> ranges;
    if (visualPolicy.enableClusterCulling && !item.lod->clusters.empty() && item.lod->indexCount > 0) {
      struct ClusterCandidate {
        const GpuMesh::ClusterGpu* cluster = nullptr;
        cad::ProjectedSphere projected{};
        float depth01 = 1.0F;
        float score = 0.0F;
      };
      const bool enableFieldIndexBudget =
          item.aggressiveLodCap &&
          visualPolicy.fieldClusterMaxVisibleIndices > 0 &&
          autoQualityEnabled_ &&
          (adaptiveLodBias > 0 || frameAverageMs > static_cast<double>(perfTargetFrameMs_));
      const bool needProjected =
          visualPolicy.enableOcclusionCulling ||
          visualPolicy.minClusterScreenRadiusPx > 0.0F ||
          enableFieldIndexBudget;
      std::vector<ClusterCandidate> candidates;
      candidates.reserve(item.lod->clusters.size());
      for (const auto& cluster : item.lod->clusters) {
        ++totalClusters;
        if (cluster.indexCount == 0) {
          continue;
        }
        const glm::vec3 worldCenter = glm::vec3(item.model * glm::vec4(cluster.boundsCenter, 1.0F));
        const float worldRadius = std::max(cluster.boundsRadius * std::max(item.worldMaxScale, 1e-4F), 1e-4F);
        if (visualPolicy.enableFrustumCulling &&
            !cad::IsSphereVisible(frustumPlanes, worldCenter, worldRadius, std::max(0.0F, visualPolicy.frustumCullMarginM))) {
          ++culledClusters;
          continue;
        }
        ClusterCandidate candidate;
        candidate.cluster = &cluster;
        candidate.depth01 = ComputeDepth01(context.viewProjection, worldCenter);
        if (needProjected) {
          candidate.projected = cad::ProjectSphereToViewport(
              worldCenter,
              worldRadius,
              context.viewProjection,
              context.viewportWidth,
              context.viewportHeight);
        }
        candidates.push_back(candidate);
      }

      if (visualPolicy.enableOcclusionCulling) {
        std::sort(candidates.begin(), candidates.end(), [](const ClusterCandidate& a, const ClusterCandidate& b) {
          return a.depth01 < b.depth01;
        });
      }

      std::vector<ClusterCandidate> surviving;
      surviving.reserve(candidates.size());
      for (const auto& candidate : candidates) {
        if (candidate.cluster == nullptr) {
          continue;
        }
        const bool occluded = visualPolicy.enableOcclusionCulling &&
                              candidate.projected.valid &&
                              occlusionBuffer.IsOccluded(
                                  candidate.projected,
                                  visualPolicy.hzbDepthMargin,
                                  visualPolicy.hzbMaxCullRadiusPx);
        if (occluded) {
          ++culledClusters;
          continue;
        }

        if (visualPolicy.minClusterScreenRadiusPx > 0.0F &&
            candidate.projected.valid &&
            candidate.projected.radiusPx < visualPolicy.minClusterScreenRadiusPx) {
          ++culledClusters;
          continue;
        }

        surviving.push_back(candidate);

        if (visualPolicy.enableOcclusionCulling && candidate.projected.valid) {
          occlusionBuffer.SubmitOccluder(candidate.projected);
        }
      }

      if (enableFieldIndexBudget && surviving.size() > 1) {
        std::uint64_t totalVisibleIndices = 0ULL;
        for (const auto& candidate : surviving) {
          if (candidate.cluster == nullptr) {
            continue;
          }
          totalVisibleIndices += static_cast<std::uint64_t>(candidate.cluster->indexCount);
        }
        const std::uint64_t indexBudget = static_cast<std::uint64_t>(visualPolicy.fieldClusterMaxVisibleIndices);
        if (totalVisibleIndices > indexBudget) {
          for (auto& candidate : surviving) {
            const float radiusScore = candidate.projected.valid
                                          ? candidate.projected.radiusPx
                                          : (candidate.cluster != nullptr ? candidate.cluster->boundsRadius : 0.0F);
            const float depthWeight = 1.0F / std::max(0.08F, candidate.depth01);
            candidate.score = radiusScore * depthWeight;
          }
          std::sort(surviving.begin(), surviving.end(), [](const ClusterCandidate& a, const ClusterCandidate& b) {
            return a.score > b.score;
          });

          std::vector<ClusterCandidate> budgeted;
          budgeted.reserve(surviving.size());
          std::uint64_t usedIndices = 0ULL;
          for (const auto& candidate : surviving) {
            if (candidate.cluster == nullptr) {
              continue;
            }
            const std::uint64_t next = usedIndices + static_cast<std::uint64_t>(candidate.cluster->indexCount);
            if (!budgeted.empty() && next > indexBudget) {
              continue;
            }
            budgeted.push_back(candidate);
            usedIndices = next;
          }
          if (budgeted.empty()) {
            budgeted.push_back(surviving.front());
          }
          culledClusters += surviving.size() - budgeted.size();
          surviving.swap(budgeted);
        }
      }

      std::sort(surviving.begin(), surviving.end(), [](const ClusterCandidate& a, const ClusterCandidate& b) {
        const std::uint32_t aFirst = (a.cluster != nullptr) ? a.cluster->firstIndex : 0U;
        const std::uint32_t bFirst = (b.cluster != nullptr) ? b.cluster->firstIndex : 0U;
        return aFirst < bFirst;
      });
      for (const auto& candidate : surviving) {
        if (candidate.cluster == nullptr) {
          continue;
        }
        const std::uint32_t expectedNext = !ranges.empty() ? (ranges.back().firstIndex + ranges.back().indexCount) : 0U;
        if (!ranges.empty() &&
            candidate.cluster->firstIndex >= expectedNext &&
            (candidate.cluster->firstIndex - expectedNext) <= clusterMergeGap) {
          ranges.back().indexCount += (candidate.cluster->firstIndex - expectedNext) + candidate.cluster->indexCount;
          ranges.back().depth01 = std::min(ranges.back().depth01, candidate.depth01);
        } else {
          ranges.push_back(DrawRange{
              .firstIndex = candidate.cluster->firstIndex,
              .indexCount = candidate.cluster->indexCount,
              .depth01 = candidate.depth01,
          });
        }
      }
    } else if (item.lod->indexCount > 0) {
      ranges.push_back(DrawRange{
          .firstIndex = 0U,
          .indexCount = static_cast<std::uint32_t>(std::max(item.lod->indexCount, 0)),
          .depth01 = item.depth01,
      });
      if (visualPolicy.enableOcclusionCulling) {
        const cad::ProjectedSphere projected = cad::ProjectSphereToViewport(
            item.worldBoundsCenter,
            item.worldBoundsRadius,
            context.viewProjection,
            context.viewportWidth,
            context.viewportHeight);
        if (!occlusionBuffer.IsOccluded(projected, visualPolicy.hzbDepthMargin, visualPolicy.hzbMaxCullRadiusPx)) {
          occlusionBuffer.SubmitOccluder(projected);
        }
      }
    }

    if (ranges.empty() && item.lod->indexCount > 0) {
      ranges.push_back(DrawRange{
          .firstIndex = 0U,
          .indexCount = static_cast<std::uint32_t>(std::max(item.lod->indexCount, 0)),
          .depth01 = item.depth01,
      });
    }
    if (ranges.empty() && item.lod->vertexCount > 0) {
      ranges.push_back(DrawRange{
          .firstIndex = 0U,
          .indexCount = 0U,
          .depth01 = item.depth01,
      });
    }
    if (ranges.empty()) {
      continue;
    }

    drawCalls.push_back(DrawCall{
        .mesh = item.mesh,
        .lod = item.lod,
        .shadowLod = item.shadowLod,
        .albedoTexture = item.albedoTexture,
        .normalTexture = item.normalTexture,
        .model = item.model,
        .color = item.color,
        .roughness = item.roughness,
        .metallic = item.metallic,
        .normalStrength = item.normalStrength,
        .depth01 = item.depth01,
        .useAssetColor = item.useAssetColor,
        .fastShading = item.fastShading,
        .wireframe = item.wireframe,
        .mirroredWinding = item.mirroredWinding,
        .ranges = std::move(ranges),
    });
  }

  if (drawCalls.empty()) {
    backend_->SetBlendEnabled(true);
    backend_->SetWireframeMode(false);
    return;
  }

  if (context.diagnosticsWriter != nullptr && pendingLoads_.empty() && pendingGpuUploads_.empty()) {
    context.diagnosticsWriter->MarkSceneReady();
  }

  if (context.diagnosticsWriter != nullptr) {
    double totalIndices = 0.0;
    std::size_t totalRanges = 0;
    for (const auto& call : drawCalls) {
      for (const auto& range : call.ranges) {
        if (range.indexCount > 0) {
          totalIndices += static_cast<double>(range.indexCount);
        } else if (call.lod != nullptr) {
          totalIndices += static_cast<double>(std::max(call.lod->vertexCount, 0));
        }
      }
      totalRanges += call.ranges.size();
    }
    context.diagnosticsWriter->RecordCounter("cad.draw.items", static_cast<double>(drawCalls.size()));
    context.diagnosticsWriter->RecordCounter("cad.draw.ranges", static_cast<double>(totalRanges));
    context.diagnosticsWriter->RecordCounter("cad.draw.indices", totalIndices);
    context.diagnosticsWriter->RecordCounter("cad.cluster.total", static_cast<double>(totalClusters));
    context.diagnosticsWriter->RecordCounter("cad.cluster.culled", static_cast<double>(culledClusters));
    context.diagnosticsWriter->RecordCounter("cad.cluster.merge_gap_indices", static_cast<double>(clusterMergeGap));
    context.diagnosticsWriter->RecordCounter(
        "cad.cluster.field_index_budget",
        autoQualityEnabled_ ? static_cast<double>(std::max(0, visualPolicy.fieldClusterMaxVisibleIndices)) : 0.0);
    context.diagnosticsWriter->RecordCounter(
        "cad.cluster.min_screen_radius_px",
        static_cast<double>(std::max(0.0F, visualPolicy.minClusterScreenRadiusPx)));
    context.diagnosticsWriter->RecordCounter("cad.mdi.enabled", allowMdi ? 1.0 : 0.0);
  }

  glm::vec3 sceneCenter{0.0F, 0.0F, 0.0F};
  float sceneRadius = 1.0F;
  {
    glm::vec3 minP{std::numeric_limits<float>::max()};
    glm::vec3 maxP{std::numeric_limits<float>::lowest()};
    for (const auto& call : drawCalls) {
      const glm::vec3 c = glm::vec3(call.model * glm::vec4(call.mesh->boundsCenter, 1.0F));
      const glm::vec3 scaleAxisX{call.model[0][0], call.model[0][1], call.model[0][2]};
      const glm::vec3 scaleAxisY{call.model[1][0], call.model[1][1], call.model[1][2]};
      const glm::vec3 scaleAxisZ{call.model[2][0], call.model[2][1], call.model[2][2]};
      const float maxScale =
          std::max(glm::length(scaleAxisX), std::max(glm::length(scaleAxisY), glm::length(scaleAxisZ)));
      const float r = std::max(call.mesh->boundsRadius * std::max(maxScale, 1e-4F), 0.1F);
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
  bool shadowCacheReused = false;

  if (nearShadowReady) {
    struct ShadowBatch {
      const GpuMesh::LodGpu* lod = nullptr;
      bool mirroredWinding = false;
      std::vector<InstanceGpuData> instances;
    };
    std::vector<ShadowBatch> shadowBatches;
    shadowBatches.reserve(drawCalls.size());
    int shadowCasterSkipped = 0;
    for (const auto& call : drawCalls) {
      const int shadowIndexCount = (call.shadowLod != nullptr) ? std::max(call.shadowLod->indexCount, 0) : 0;
      if (shadowCasterMaxIndices_ > 0 && shadowIndexCount > shadowCasterMaxIndices_) {
        ++shadowCasterSkipped;
        continue;
      }
      auto it = std::find_if(shadowBatches.begin(), shadowBatches.end(), [&](const ShadowBatch& batch) {
        return batch.lod == call.shadowLod && batch.mirroredWinding == call.mirroredWinding;
      });
      if (it == shadowBatches.end()) {
        ShadowBatch batch;
        batch.lod = call.shadowLod;
        batch.mirroredWinding = call.mirroredWinding;
        batch.instances.push_back(ToInstanceData(call.model));
        shadowBatches.push_back(std::move(batch));
      } else {
        it->instances.push_back(ToInstanceData(call.model));
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
      std::uint64_t shadowFingerprint = kFnvOffsetBasis;
      HashMixU64(shadowFingerprint, static_cast<std::uint64_t>(shadowMapSize_));
      HashMixU64(shadowFingerprint, farShadowReady ? 1ULL : 0ULL);
      for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
          HashMixFloat(shadowFingerprint, lightViewProjection[col][row]);
          HashMixFloat(shadowFingerprint, lightViewProjectionFar[col][row]);
        }
      }
      for (const auto& batch : shadowBatches) {
        HashMixU64(shadowFingerprint, static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(batch.lod)));
        HashMixU64(shadowFingerprint, batch.mirroredWinding ? 1ULL : 0ULL);
        HashMixU64(shadowFingerprint, static_cast<std::uint64_t>(batch.instances.size()));
        for (const auto& instance : batch.instances) {
          HashMixFloat(shadowFingerprint, instance.modelRow0.x);
          HashMixFloat(shadowFingerprint, instance.modelRow0.y);
          HashMixFloat(shadowFingerprint, instance.modelRow0.z);
          HashMixFloat(shadowFingerprint, instance.modelRow0.w);
          HashMixFloat(shadowFingerprint, instance.modelRow1.x);
          HashMixFloat(shadowFingerprint, instance.modelRow1.y);
          HashMixFloat(shadowFingerprint, instance.modelRow1.z);
          HashMixFloat(shadowFingerprint, instance.modelRow1.w);
          HashMixFloat(shadowFingerprint, instance.modelRow2.x);
          HashMixFloat(shadowFingerprint, instance.modelRow2.y);
          HashMixFloat(shadowFingerprint, instance.modelRow2.z);
          HashMixFloat(shadowFingerprint, instance.modelRow2.w);
          HashMixFloat(shadowFingerprint, instance.modelRow3.x);
          HashMixFloat(shadowFingerprint, instance.modelRow3.y);
          HashMixFloat(shadowFingerprint, instance.modelRow3.z);
          HashMixFloat(shadowFingerprint, instance.modelRow3.w);
        }
      }

      const bool shouldRenderShadowMaps =
          !shadowCacheValid_ ||
          shadowFingerprint != shadowCacheFingerprint_;
      shadowCacheReused = !shouldRenderShadowMaps;

      if (shouldRenderShadowMaps) {
        renderShadowPass(shadowFbo_, lightViewProjection);
        if (farShadowReady) {
          renderShadowPass(shadowFboFar_, lightViewProjectionFar);
        }
        shadowCacheFingerprint_ = shadowFingerprint;
        shadowCacheValid_ = true;
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
      shadowCacheValid_ = false;
      shadowCacheFingerprint_ = 0ULL;
    }

    if (context.diagnosticsWriter != nullptr) {
      context.diagnosticsWriter->RecordCounter("cad.shadow.cache_reused", shadowCacheReused ? 1.0 : 0.0);
    }

    glCullFace(GL_BACK);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, context.viewportWidth, context.viewportHeight);
  } else {
    shadowCacheValid_ = false;
    shadowCacheFingerprint_ = 0ULL;
  }

  auto drawIndexedRangesSingleInstance = [&](const std::vector<DrawRange>& ranges) {
    if (ranges.empty()) {
      return;
    }
    if (!allowMdi || cadIndirectDrawBuffer_.Get() == 0 || ranges.size() <= 1) {
      for (const auto& range : ranges) {
        if (range.indexCount == 0) {
          continue;
        }
        backend_->DrawIndexedTrianglesInstancedRange(
            static_cast<int>(range.indexCount),
            1,
            static_cast<std::size_t>(range.firstIndex));
      }
      return;
    }

    std::vector<DrawElementsIndirectCommand> commands;
    commands.reserve(ranges.size());
    for (const auto& range : ranges) {
      if (range.indexCount == 0) {
        continue;
      }
      commands.push_back(DrawElementsIndirectCommand{
          .count = range.indexCount,
          .instanceCount = 1U,
          .firstIndex = range.firstIndex,
          .baseVertex = 0U,
          .baseInstance = 0U,
      });
    }
    if (commands.empty()) {
      return;
    }

    glBindBuffer(GL_DRAW_INDIRECT_BUFFER, cadIndirectDrawBuffer_.Get());
    glBufferData(
        GL_DRAW_INDIRECT_BUFFER,
        static_cast<GLsizeiptr>(commands.size() * sizeof(DrawElementsIndirectCommand)),
        commands.data(),
        GL_STREAM_DRAW);
    glMultiDrawElementsIndirect(
        GL_TRIANGLES,
        GL_UNSIGNED_INT,
        nullptr,
        static_cast<GLsizei>(commands.size()),
        0);
    const GLenum mdiError = glGetError();
    glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
    if (mdiError != GL_NO_ERROR) {
      mdiRuntimeDisabled = true;
      for (const auto& range : ranges) {
        if (range.indexCount == 0) {
          continue;
        }
        backend_->DrawIndexedTrianglesInstancedRange(
            static_cast<int>(range.indexCount),
            1,
            static_cast<std::size_t>(range.firstIndex));
      }
    }
  };

  if (visualPolicy.enableDepthPrepass && depthPrepassShader_.Get() != 0 && renderPass_ == RenderPass::Opaque) {
    std::vector<std::size_t> prepassOrder(drawCalls.size());
    for (std::size_t i = 0; i < prepassOrder.size(); ++i) {
      prepassOrder[i] = i;
    }
    std::sort(prepassOrder.begin(), prepassOrder.end(), [&](const std::size_t a, const std::size_t b) {
      return drawCalls[a].depth01 < drawCalls[b].depth01;
    });

    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    backend_->UseProgram(depthPrepassShader_.Get());
    backend_->SetUniformMat4(uDepthPrepassViewProjectionLoc_, &context.viewProjection[0][0]);

    for (const std::size_t callIndex : prepassOrder) {
      const auto& call = drawCalls[callIndex];
      if (call.lod == nullptr) {
        continue;
      }
      if (call.mirroredWinding) {
        glFrontFace(GL_CW);
      }
      const InstanceGpuData instanceData = ToInstanceData(call.model);
      backend_->BindVertexArray(call.lod->vao.Get());
      backend_->BindArrayBuffer(call.lod->instanceVbo.Get());
      backend_->UploadArrayBufferData(sizeof(InstanceGpuData), &instanceData, true);
      if (call.lod->indexCount > 0) {
        drawIndexedRangesSingleInstance(call.ranges);
      } else {
        backend_->DrawTrianglesInstanced(call.lod->vertexCount, 1);
      }
      backend_->BindVertexArray(0);
      if (call.mirroredWinding) {
        glFrontFace(GL_CCW);
      }
    }
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_FALSE);
    glDepthFunc(GL_LEQUAL);
  } else {
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LEQUAL);
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

  std::vector<std::size_t> drawOrder(drawCalls.size());
  for (std::size_t i = 0; i < drawOrder.size(); ++i) {
    drawOrder[i] = i;
  }
  std::sort(drawOrder.begin(), drawOrder.end(), [&](const std::size_t a, const std::size_t b) {
    const auto keyA = std::tuple{
        drawCalls[a].wireframe,
        drawCalls[a].fastShading,
        drawCalls[a].useAssetColor,
        reinterpret_cast<std::uintptr_t>(drawCalls[a].albedoTexture),
        reinterpret_cast<std::uintptr_t>(drawCalls[a].normalTexture),
        FloatBits(drawCalls[a].color.r),
        FloatBits(drawCalls[a].color.g),
        FloatBits(drawCalls[a].color.b),
        FloatBits(drawCalls[a].color.a),
        FloatBits(drawCalls[a].roughness),
        FloatBits(drawCalls[a].metallic),
        FloatBits(drawCalls[a].normalStrength),
        reinterpret_cast<std::uintptr_t>(drawCalls[a].lod),
        drawCalls[a].mirroredWinding,
        drawCalls[a].depth01};
    const auto keyB = std::tuple{
        drawCalls[b].wireframe,
        drawCalls[b].fastShading,
        drawCalls[b].useAssetColor,
        reinterpret_cast<std::uintptr_t>(drawCalls[b].albedoTexture),
        reinterpret_cast<std::uintptr_t>(drawCalls[b].normalTexture),
        FloatBits(drawCalls[b].color.r),
        FloatBits(drawCalls[b].color.g),
        FloatBits(drawCalls[b].color.b),
        FloatBits(drawCalls[b].color.a),
        FloatBits(drawCalls[b].roughness),
        FloatBits(drawCalls[b].metallic),
        FloatBits(drawCalls[b].normalStrength),
        reinterpret_cast<std::uintptr_t>(drawCalls[b].lod),
        drawCalls[b].mirroredWinding,
        drawCalls[b].depth01};
    return keyA < keyB;
  });

  for (const std::size_t callIndex : drawOrder) {
    const auto& call = drawCalls[callIndex];
    if (call.lod == nullptr || call.ranges.empty()) {
      continue;
    }
    MaterialInstance material;
    material.definition.baseColor = call.color;
    material.definition.wireframe = call.wireframe;
    materialPipeline_.Apply(material);
    backend_->SetUniform1f(uRoughnessLoc_, call.roughness);
    backend_->SetUniform1f(uMetallicLoc_, call.metallic);
    backend_->SetUniform1f(uNormalStrengthLoc_, call.normalStrength);
    backend_->SetUniform1f(uTriplanarScaleLoc_, triplanarScale_);
    backend_->SetUniform1f(uUseAssetColorLoc_, call.useAssetColor ? 1.0F : 0.0F);
    backend_->SetUniform1i(uShadingModeLoc_, call.fastShading ? 1 : 0);
    const bool useNormalMap = (call.normalTexture != nullptr) && !call.fastShading;
    backend_->SetUniform1i(uHasAlbedoMapLoc_, call.albedoTexture != nullptr ? 1 : 0);
    backend_->SetUniform1i(uHasNormalMapLoc_, useNormalMap ? 1 : 0);
    backend_->SetActiveTextureUnit(5);
    backend_->BindTexture2D(call.albedoTexture != nullptr ? call.albedoTexture->texture.Get() : 0);
    backend_->SetUniform1i(uAlbedoMapLoc_, 5);
    backend_->SetActiveTextureUnit(6);
    backend_->BindTexture2D(useNormalMap ? call.normalTexture->texture.Get() : 0);
    backend_->SetUniform1i(uNormalMapLoc_, 6);

    if (call.mirroredWinding) {
      glFrontFace(GL_CW);
    }
    const InstanceGpuData instanceData = ToInstanceData(call.model);
    backend_->BindVertexArray(call.lod->vao.Get());
    backend_->BindArrayBuffer(call.lod->instanceVbo.Get());
    backend_->UploadArrayBufferData(sizeof(InstanceGpuData), &instanceData, true);
    if (call.lod->indexCount > 0) {
      drawIndexedRangesSingleInstance(call.ranges);
    } else {
      backend_->DrawTrianglesInstanced(call.lod->vertexCount, 1);
    }
    backend_->BindVertexArray(0);
    if (call.mirroredWinding) {
      glFrontFace(GL_CCW);
    }
  }

  glDepthMask(GL_TRUE);
  glDepthFunc(GL_LEQUAL);
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
