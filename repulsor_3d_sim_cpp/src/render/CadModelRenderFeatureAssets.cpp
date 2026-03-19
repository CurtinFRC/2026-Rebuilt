#include "repulsor3d/render/CadModelRenderFeature.hpp"

#include <stb_image.h>

#include <chrono>
#include <future>
#include <iostream>
#include <sstream>

#include "repulsor3d/Renderer.hpp"
#include "repulsor3d/render/cad/CadPolicies.hpp"
#include "cad/feature/CadModelRenderFeatureInternals.hpp"

namespace repulsor3d {

using cad::CadLodPolicy;
using cad::CadVisualPolicy;
using cad::GetEnvBool;
using cad::LoadCadLodPolicy;
using cad::LoadCadVisualPolicy;
using cadfeature::BuildShadowProxyMesh;
using cadfeature::BuildMeshClusters;
using cadfeature::IndexedMesh;
using cadfeature::PreparedCpuMeshBlob;
using cadfeature::StorePreparedCpuMeshCache;
using cadfeature::TryLoadPreparedCpuMeshCache;

namespace {

std::string MeshHandleId(const std::string& assetPath) {
  return "cad.mesh:" + assetPath;
}

std::string TextureHandleId(const std::string& assetPath) {
  return "cad.texture:" + assetPath;
}

}  // namespace

const CadModelRenderFeature::GpuMesh* CadModelRenderFeature::GetOrLoadMesh(const std::string& assetPath) {
  const auto cached = meshCache_.find(assetPath);
  if (cached != meshCache_.end()) {
    cacheLifetimeManager_.Touch(ResourceClass::Mesh, MeshHandleId(assetPath));
    return &cached->second;
  }
  if (failedLoads_.find(assetPath) != failedLoads_.end()) {
    if (!GetEnvBool("CAD_AUTO_RETRY_FAILED_LOADS", false)) {
      return nullptr;
    }
    failedLoads_.erase(assetPath);
  }
  if (geometryProvider_ == nullptr) {
    return nullptr;
  }

  const auto pending = pendingLoads_.find(assetPath);
  if (pending != pendingLoads_.end() &&
      pending->second.future.valid() &&
      pending->second.future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
    PreparedCpuMesh prepared;
    try {
      prepared = pending->second.future.get();
    } catch (...) {
      prepared = PreparedCpuMesh{};
    }
    if (pending->second.status != nullptr) {
      pending->second.status->completed = true;
      pending->second.status->progress = 1.0F;
    }
    pendingLoads_.erase(pending);
    if (prepared.lods.empty()) {
      failedLoads_.insert(assetPath);
      std::cerr << "CAD load failed: " << assetPath << "\n";
      return nullptr;
    }

    PendingGpuUpload upload;
    upload.prepared = std::move(prepared);
    upload.gpu.boundsCenter = upload.prepared.boundsCenter;
    upload.gpu.boundsRadius = upload.prepared.boundsRadius;
    pendingGpuUploads_[assetPath] = std::move(upload);
  }

  const auto uploadIt = pendingGpuUploads_.find(assetPath);
  if (uploadIt != pendingGpuUploads_.end()) {
    if (backend_ == nullptr) {
      return nullptr;
    }
    auto& upload = uploadIt->second;
    const int activeUploadBudget =
        upload.prepared.loadedFromPreparedCache ? uploadBudgetPerFrameCacheHit_ : uploadBudgetPerFrame_;
    int uploadsForThisAsset = 0;
    while (uploadsThisFrame_ < activeUploadBudget && uploadsForThisAsset < activeUploadBudget) {
      if (upload.nextLod < upload.prepared.lods.size()) {
        const auto& cpuLod = upload.prepared.lods[upload.nextLod];
        if (cpuLod.vertices.empty()) {
          ++upload.nextLod;
          continue;
        }

        GpuMesh::LodGpu gpuLod;
        if (!UploadSingleLod(cpuLod, gpuLod, *backend_)) {
          failedLoads_.insert(assetPath);
          DestroyMesh(upload.gpu);
          pendingGpuUploads_.erase(assetPath);
          std::cerr << "CAD upload failed: " << assetPath << "\n";
          return nullptr;
        }
        upload.gpu.lods.push_back(std::move(gpuLod));
        ++upload.nextLod;
        ++uploadsThisFrame_;
        ++uploadsForThisAsset;
        if (!upload.prepared.loadedFromPreparedCache) {
          break;  // spread heavy uploads across frames for cold loads
        }
        continue;
      }

      if (!upload.shadowProxyUploaded && upload.prepared.shadowProxy.has_value()) {
        GpuMesh::LodGpu gpuShadow;
        if (!UploadSingleLod(upload.prepared.shadowProxy.value(), gpuShadow, *backend_)) {
          failedLoads_.insert(assetPath);
          DestroyMesh(upload.gpu);
          pendingGpuUploads_.erase(assetPath);
          std::cerr << "CAD shadow-proxy upload failed: " << assetPath << "\n";
          return nullptr;
        }
        upload.gpu.shadowProxy = std::move(gpuShadow);
        upload.shadowProxyUploaded = true;
        ++uploadsThisFrame_;
        ++uploadsForThisAsset;
        if (!upload.prepared.loadedFromPreparedCache) {
          break;
        }
        continue;
      }
      break;
    }

    const bool hasPendingLodUploads = upload.nextLod < upload.prepared.lods.size();
    const bool hasPendingShadowProxy =
        !upload.shadowProxyUploaded && upload.prepared.shadowProxy.has_value();
    if (hasPendingLodUploads || hasPendingShadowProxy) {
      return nullptr;
    }

    auto [it, inserted] = meshCache_.emplace(assetPath, std::move(upload.gpu));
    pendingGpuUploads_.erase(assetPath);
    if (!inserted) {
      cacheLifetimeManager_.Touch(ResourceClass::Mesh, MeshHandleId(assetPath));
      return &it->second;
    }
    const std::size_t meshBytes = EstimateGpuMeshBytes(it->second);
    meshCacheBytes_[assetPath] = meshBytes;
    cacheLifetimeManager_.Register(ResourceClass::Mesh, MeshHandleId(assetPath), meshBytes);
    EnforceGpuCacheBudgets();

    std::ostringstream lodSummary;
    for (std::size_t i = 0; i < it->second.lods.size(); ++i) {
      if (i > 0) {
        lodSummary << ", ";
      }
      lodSummary << "L" << i << "(v=" << it->second.lods[i].vertexCount
                 << ",i=" << it->second.lods[i].indexCount << ")";
    }
    std::cerr << "CAD loaded: " << assetPath
              << " (lods=" << it->second.lods.size()
              << ", " << lodSummary.str()
              << (it->second.shadowProxy.has_value()
                      ? std::string(", Shadow(v=") +
                            std::to_string(it->second.shadowProxy->vertexCount) +
                            ",i=" + std::to_string(it->second.shadowProxy->indexCount) + ")"
                      : "")
              << ", centerXY=(" << it->second.boundsCenter.x << ", " << it->second.boundsCenter.y << ")"
              << ", radius=" << it->second.boundsRadius << ")\n";
    return &it->second;
  }

  if (pendingLoads_.find(assetPath) != pendingLoads_.end()) {
    return nullptr;
  }

  IMeshProvider* provider = meshProvider_.get();
  auto status = std::make_shared<PendingLoad::Status>();
  status->progress = 0.02F;
  {
    std::scoped_lock lock(status->stageMutex);
    status->stage = "queued";
  }
  std::packaged_task<PreparedCpuMesh()> loadTask(
      [provider, assetPath, status]() {
            auto setStage = [&](const char* stage, const float progress) {
              if (status != nullptr) {
                status->progress = progress;
                std::scoped_lock lock(status->stageMutex);
                status->stage = stage;
              }
            };
            auto wasCancelled = [&]() {
              return status != nullptr && status->cancelRequested.load();
            };

            const CadLodPolicy& policy = LoadCadLodPolicy();
            setStage("cache_lookup", 0.10F);
            PreparedCpuMeshBlob cachedBlob;
            if (TryLoadPreparedCpuMeshCache(assetPath, policy, cachedBlob)) {
              setStage("cache_hit", 0.70F);
              const CadVisualPolicy& visualPolicy = LoadCadVisualPolicy();
              const bool rebuildClustersOnCacheHit = GetEnvBool("CAD_CACHE_HIT_REBUILD_CLUSTERS", false);
              PreparedCpuMesh prepared;
              prepared.loadedFromPreparedCache = true;
              prepared.boundsCenter = cachedBlob.boundsCenter;
              prepared.boundsRadius = cachedBlob.boundsRadius;
              prepared.materialHints.roughness = cachedBlob.roughnessHint;
              prepared.materialHints.metallic = cachedBlob.metallicHint;
              prepared.lods.reserve(cachedBlob.lods.size());
              for (auto& cachedLod : cachedBlob.lods) {
                PreparedCpuMesh::LodCpu lod;
                lod.vertices = std::move(cachedLod.vertices);
                lod.indices = std::move(cachedLod.indices);
                if (visualPolicy.enableClusterCulling && rebuildClustersOnCacheHit) {
                  std::vector<cadfeature::MeshCluster> clusters = BuildMeshClusters(
                      lod.indices,
                      lod.vertices,
                      static_cast<std::size_t>(std::max(visualPolicy.clusterTargetIndices, 3)),
                      static_cast<std::size_t>(std::max(visualPolicy.clusterMinIndices, 3)),
                      visualPolicy.optimizeVertexCache);
                  lod.clusters.reserve(clusters.size());
                  for (const auto& cluster : clusters) {
                    lod.clusters.push_back(PreparedCpuMesh::ClusterCpu{
                        .firstIndex = cluster.firstIndex,
                        .indexCount = cluster.indexCount,
                        .boundsCenter = cluster.boundsCenter,
                        .boundsRadius = cluster.boundsRadius,
                    });
                  }
                }
                prepared.lods.push_back(std::move(lod));
              }
              std::cerr << "CAD prepared cache hit: " << assetPath << "\n";
              setStage("completed", 1.00F);
              return prepared;
            }
            std::cerr << "CAD prepared cache miss: " << assetPath << "\n";
            if (wasCancelled()) {
              setStage("cancelled", 1.0F);
              return PreparedCpuMesh{};
            }

            setStage("import", 0.25F);
            PositionNormalMesh cpu;
            if (provider == nullptr || !provider->LoadPositionNormalMesh(assetPath, cpu)) {
              setStage("import_failed", 1.0F);
              if (status != nullptr) {
                status->failed = true;
              }
              return PreparedCpuMesh{};
            }
            if (wasCancelled()) {
              setStage("cancelled", 1.0F);
              return PreparedCpuMesh{};
            }

            setStage("prepare", 0.60F);
            PreparedCpuMesh prepared = PrepareCpuMesh(cpu);
            prepared.loadedFromPreparedCache = false;
            if (!prepared.lods.empty()) {
              setStage("store_cache", 0.85F);
              PreparedCpuMeshBlob blob;
              blob.boundsCenter = prepared.boundsCenter;
              blob.boundsRadius = prepared.boundsRadius;
              blob.roughnessHint = prepared.materialHints.roughness;
              blob.metallicHint = prepared.materialHints.metallic;
              blob.lods.reserve(prepared.lods.size());
              for (const auto& lod : prepared.lods) {
                PreparedCpuMeshBlob::LodBlob cachedLod;
                cachedLod.vertices = lod.vertices;
                cachedLod.indices = lod.indices;
                blob.lods.push_back(std::move(cachedLod));
              }
              StorePreparedCpuMeshCache(assetPath, policy, blob);
            }
            if (prepared.lods.empty() && status != nullptr) {
              status->failed = true;
            }
            setStage("completed", 1.00F);
            return prepared;
          });
  std::future<PreparedCpuMesh> future = loadTask.get_future();
  auto cancellationToken = TaskScheduler::CancellationToken::Create();
  EnqueueLoadTask(std::move(loadTask), TaskScheduler::TaskPriority::Low, cancellationToken);
  pendingLoads_.emplace(assetPath, PendingLoad{std::move(future), status, cancellationToken});
  std::cerr << "CAD loading started: " << assetPath << "\n";
  return nullptr;
}

const CadModelRenderFeature::GpuTexture* CadModelRenderFeature::GetOrLoadTexture(const std::string& assetPath) {
  if (assetPath.empty()) {
    return nullptr;
  }
  const auto cached = textureCache_.find(assetPath);
  if (cached != textureCache_.end()) {
    cacheLifetimeManager_.Touch(ResourceClass::Texture, TextureHandleId(assetPath));
    return &cached->second;
  }
  if (failedTextureLoads_.contains(assetPath)) {
    return nullptr;
  }
  if (renderer_ == nullptr || backend_ == nullptr) {
    return nullptr;
  }

  const std::string resolved = renderer_->GetAssetResolver().ResolveFilePath(assetPath);
  if (resolved.empty()) {
    failedTextureLoads_.insert(assetPath);
    return nullptr;
  }

  stbi_set_flip_vertically_on_load(0);
  int width = 0;
  int height = 0;
  int channels = 0;
  unsigned char* pixels = stbi_load(resolved.c_str(), &width, &height, &channels, STBI_rgb_alpha);
  if (pixels == nullptr || width <= 0 || height <= 0) {
    if (pixels != nullptr) {
      stbi_image_free(pixels);
    }
    failedTextureLoads_.insert(assetPath);
    return nullptr;
  }

  GpuTexture texture;
  texture.texture.Set(backend_->CreateTexture2D());
  texture.width = width;
  texture.height = height;
  if (texture.texture.Get() == 0) {
    stbi_image_free(pixels);
    failedTextureLoads_.insert(assetPath);
    return nullptr;
  }
  backend_->BindTexture2D(texture.texture.Get());
  backend_->SetTexture2DLinearMipmapClamp();
  backend_->UploadTexture2DRgba8(width, height, pixels);
  backend_->GenerateTexture2DMipmaps();
  backend_->BindTexture2D(0);
  stbi_image_free(pixels);

  auto [it, inserted] = textureCache_.emplace(assetPath, std::move(texture));
  if (!inserted) {
    cacheLifetimeManager_.Touch(ResourceClass::Texture, TextureHandleId(assetPath));
    return &it->second;
  }
  const std::size_t textureBytes = static_cast<std::size_t>(std::max(0, width)) *
                                   static_cast<std::size_t>(std::max(0, height)) * 4ULL;
  textureCacheBytes_[assetPath] = textureBytes;
  cacheLifetimeManager_.Register(ResourceClass::Texture, TextureHandleId(assetPath), textureBytes);
  EnforceGpuCacheBudgets();
  std::cerr << "CAD texture loaded: " << assetPath << " (" << width << "x" << height << ")\n";
  return &it->second;
}

bool CadModelRenderFeature::EvictMeshAsset(const std::string& assetPath) {
  if (assetPath.empty()) {
    return false;
  }
  const auto it = meshCache_.find(assetPath);
  if (it == meshCache_.end()) {
    return false;
  }
  DestroyMesh(it->second);
  meshCache_.erase(it);
  meshCacheBytes_.erase(assetPath);
  failedLoads_.erase(assetPath);
  pendingLoads_.erase(assetPath);
  pendingGpuUploads_.erase(assetPath);
  std::cerr << "CAD mesh evicted: " << assetPath << "\n";
  return true;
}

bool CadModelRenderFeature::EvictTextureAsset(const std::string& assetPath) {
  if (assetPath.empty()) {
    return false;
  }
  const auto it = textureCache_.find(assetPath);
  if (it == textureCache_.end()) {
    return false;
  }
  it->second.texture.Reset();
  textureCache_.erase(it);
  textureCacheBytes_.erase(assetPath);
  failedTextureLoads_.erase(assetPath);
  std::cerr << "CAD texture evicted: " << assetPath << "\n";
  return true;
}

void CadModelRenderFeature::EnforceGpuCacheBudgets() {
  while (cacheLifetimeManager_.IsOverBudget(ResourceClass::Mesh)) {
    const bool evicted = cacheLifetimeManager_.EvictOne(
        ResourceClass::Mesh,
        [this](const ResourceLifetimeManager::EvictionCandidate& candidate) {
          if (candidate.handleId.rfind("cad.mesh:", 0) != 0) {
            return false;
          }
          const std::string assetPath = candidate.handleId.substr(std::string("cad.mesh:").size());
          return EvictMeshAsset(assetPath);
        });
    if (!evicted) {
      break;
    }
  }

  while (cacheLifetimeManager_.IsOverBudget(ResourceClass::Texture)) {
    const bool evicted = cacheLifetimeManager_.EvictOne(
        ResourceClass::Texture,
        [this](const ResourceLifetimeManager::EvictionCandidate& candidate) {
          if (candidate.handleId.rfind("cad.texture:", 0) != 0) {
            return false;
          }
          const std::string assetPath = candidate.handleId.substr(std::string("cad.texture:").size());
          return EvictTextureAsset(assetPath);
        });
    if (!evicted) {
      break;
    }
  }
}

}  // namespace repulsor3d
