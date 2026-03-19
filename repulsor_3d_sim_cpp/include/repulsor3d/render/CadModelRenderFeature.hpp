#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstddef>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include "repulsor3d/render/RenderFeature.hpp"
#include "repulsor3d/async/TaskScheduler.hpp"
#include "repulsor3d/render/backend/GlHandles.hpp"
#include "repulsor3d/render/assets/MeshProvider.hpp"
#include "repulsor3d/render/backend/RenderBackend.hpp"
#include "repulsor3d/render/geometry/GeometryProvider.hpp"
#include "repulsor3d/render/material/Material.hpp"
#include "repulsor3d/render/material/MaterialPipeline.hpp"
#include "repulsor3d/render/resources/ResourceLifetimeManager.hpp"

namespace repulsor3d {
namespace cad {
class UniformGridBroadphase;
}

class IGeometryProvider;
class Renderer;

class CadModelRenderFeature final : public IRenderFeature {
 public:
  explicit CadModelRenderFeature(
      RenderPass renderPass = RenderPass::Opaque,
      std::string featureName = "cad_opaque",
      std::vector<std::string> dependencies = {"geometry_opaque"});
  ~CadModelRenderFeature() override;

  CadModelRenderFeature(const CadModelRenderFeature&) = delete;
  CadModelRenderFeature& operator=(const CadModelRenderFeature&) = delete;

  std::string Name() const override { return featureName_; }
  std::vector<std::string> Dependencies() const override { return dependencies_; }

  bool Initialize(Renderer& renderer) override;
  void Render(const RenderFeatureContext& context, const RendererDrawApi& drawApi) override;

 private:
  class FlatLitMaterialPass final : public IMaterialPass {
   public:
    FlatLitMaterialPass(IRenderBackend& backend, int colorUniformLocation)
        : backend_(backend), colorUniformLocation_(colorUniformLocation) {}
    void Apply(const MaterialInstance& material) override;

   private:
    IRenderBackend& backend_;
    int colorUniformLocation_ = -1;
  };

  class WireframeMaterialPass final : public IMaterialPass {
   public:
    explicit WireframeMaterialPass(IRenderBackend& backend) : backend_(backend) {}
    void Apply(const MaterialInstance& material) override;

   private:
    IRenderBackend& backend_;
  };

  struct GpuMesh {
    struct ClusterGpu {
      std::uint32_t firstIndex = 0;
      std::uint32_t indexCount = 0;
      glm::vec3 boundsCenter{0.0F, 0.0F, 0.0F};
      float boundsRadius = 0.0F;
    };
    struct LodGpu {
      GlVertexArrayHandle vao;
      GlBufferHandle vbo;
      GlBufferHandle ebo;
      GlBufferHandle instanceVbo;
      int vertexCount = 0;
      int indexCount = 0;
      std::vector<ClusterGpu> clusters;
    };
    std::vector<LodGpu> lods;
    std::optional<LodGpu> shadowProxy;
    glm::vec3 boundsCenter{0.0F, 0.0F, 0.0F};
    float boundsRadius = 1.0F;
    PositionNormalMesh::MaterialHints materialHints;
  };

  struct PreparedCpuMesh {
    struct ClusterCpu {
      std::uint32_t firstIndex = 0;
      std::uint32_t indexCount = 0;
      glm::vec3 boundsCenter{0.0F, 0.0F, 0.0F};
      float boundsRadius = 0.0F;
    };
    struct LodCpu {
      std::vector<PositionNormalMesh::Vertex> vertices;
      std::vector<std::uint32_t> indices;
      std::vector<ClusterCpu> clusters;
    };
    std::vector<LodCpu> lods;
    std::optional<LodCpu> shadowProxy;
    glm::vec3 boundsCenter{0.0F, 0.0F, 0.0F};
    float boundsRadius = 1.0F;
    PositionNormalMesh::MaterialHints materialHints;
  };

  struct PendingLoad {
    struct Status {
      std::atomic<float> progress{0.0F};
      std::atomic<bool> cancelRequested{false};
      std::atomic<bool> completed{false};
      std::atomic<bool> failed{false};
      std::atomic<int> retryCount{0};
      std::string stage = "queued";
      mutable std::mutex stageMutex;
    };

    std::future<PreparedCpuMesh> future;
    std::shared_ptr<Status> status;
    TaskScheduler::CancellationToken cancellationToken;
  };

  struct PendingGpuUpload {
    PreparedCpuMesh prepared;
    GpuMesh gpu;
    std::size_t nextLod = 0;
    bool shadowProxyUploaded = false;
  };

  struct GpuTexture {
    GlTextureHandle texture;
    int width = 0;
    int height = 0;
  };

  bool CreateShader();
  static unsigned int CompileShader(unsigned int type, const char* source);
  static bool LinkShader(GlProgramHandle& program, unsigned int vs, unsigned int fs);

  static PreparedCpuMesh PrepareCpuMesh(const PositionNormalMesh& cpu);
  static bool UploadSingleLod(
      const PreparedCpuMesh::LodCpu& cpuLod,
      GpuMesh::LodGpu& gpuLod,
      IRenderBackend& backend);
  static bool UploadMesh(const PreparedCpuMesh& cpu, GpuMesh& gpu, IRenderBackend& backend);
  static void DestroyMesh(GpuMesh& gpu);
  static std::size_t EstimateGpuMeshBytes(const GpuMesh& gpu);
  static std::size_t SelectLodLevel(const GpuMesh& mesh, const glm::mat4& mvp, int viewportWidth, int viewportHeight);
  void StartLoadWorker();
  void StopLoadWorker();
  void EnqueueLoadTask(
      std::packaged_task<PreparedCpuMesh()>&& task,
      TaskScheduler::TaskPriority priority = TaskScheduler::TaskPriority::Normal,
      TaskScheduler::CancellationToken token = {});
  const GpuTexture* GetOrLoadTexture(const std::string& assetPath);
  bool EvictMeshAsset(const std::string& assetPath);
  bool EvictTextureAsset(const std::string& assetPath);
  void EnforceGpuCacheBudgets();

  const GpuMesh* GetOrLoadMesh(const std::string& assetPath);

  Renderer* renderer_ = nullptr;
  IGeometryProvider* geometryProvider_ = nullptr;
  std::unique_ptr<IMeshProvider> meshProvider_;
  IRenderBackend* backend_ = nullptr;

  GlProgramHandle shader_;
  int uMvpLoc_ = -1;
  int uModelLoc_ = -1;
  int uNormalMatrixLoc_ = -1;
  int uColorLoc_ = -1;
  int uLightDirLoc_ = -1;
  int uCameraPosLoc_ = -1;
  int uKeyLightIntensityLoc_ = -1;
  int uFillLightIntensityLoc_ = -1;
  int uAmbientStrengthLoc_ = -1;
  int uDepthCueStrengthLoc_ = -1;
  int uSpecularStrengthLoc_ = -1;
  int uRimStrengthLoc_ = -1;
  int uRoughnessLoc_ = -1;
  int uMetallicLoc_ = -1;
  int uExposureLoc_ = -1;
  int uFogDensityLoc_ = -1;
  int uSaturationLoc_ = -1;
  int uGammaLoc_ = -1;
  int uUseAssetColorLoc_ = -1;
  MaterialPipeline materialPipeline_;
  GlProgramHandle shadowShader_;
  GlProgramHandle depthPrepassShader_;
  unsigned int shadowFbo_ = 0;
  unsigned int shadowFboFar_ = 0;
  GlTextureHandle shadowDepthTexture_;
  GlTextureHandle shadowDepthTextureFar_;
  int uShadowLightMvpLoc_ = -1;
  int uDepthPrepassViewProjectionLoc_ = -1;
  int uLightViewProjectionLoc_ = -1;
  int uLightViewProjectionFarLoc_ = -1;
  int uShadowMapLoc_ = -1;
  int uShadowMapFarLoc_ = -1;
  int uShadowEnabledLoc_ = -1;
  int uShadowStrengthLoc_ = -1;
  int uShadowPcfRadiusLoc_ = -1;
  int uShadowCascadeCountLoc_ = -1;
  int uShadowCascadeSplitMLoc_ = -1;
  int uAlbedoMapLoc_ = -1;
  int uNormalMapLoc_ = -1;
  int uHasAlbedoMapLoc_ = -1;
  int uHasNormalMapLoc_ = -1;
  int uShadingModeLoc_ = -1;
  int uNormalStrengthLoc_ = -1;
  int uTriplanarScaleLoc_ = -1;
  int shadowMapSize_ = 2048;
  bool shadowEnabled_ = true;
  float shadowStrength_ = 0.55F;
  int shadowPcfRadius_ = 2;
  int shadowCascadeCount_ = 1;
  float shadowCascadeSplitM_ = 12.0F;
  bool initialized_ = false;
  GlBufferHandle cadIndirectDrawBuffer_;
  std::unordered_map<std::string, GpuMesh> meshCache_;
  std::unordered_map<std::string, GpuTexture> textureCache_;
  std::unordered_set<std::string> failedTextureLoads_;
  std::unordered_map<std::string, PendingLoad> pendingLoads_;
  std::unordered_map<std::string, PendingGpuUpload> pendingGpuUploads_;
  std::unordered_set<std::string> failedLoads_;
  TaskScheduler loadScheduler_;
  std::unique_ptr<cad::UniformGridBroadphase> broadphaseCache_;
  float broadphaseCacheCellSizeM_ = -1.0F;
  std::uint64_t broadphaseCacheFingerprint_ = 0ULL;
  int uploadBudgetPerFrame_ = 1;
  int uploadsThisFrame_ = 0;
  float triplanarScale_ = 1.0F;
  bool autoQualityEnabled_ = false;
  float perfTargetFrameMs_ = 16.7F;
  int maxAutoLodBias_ = 0;
  int shadowLodBias_ = 1;
  int shadowCasterMaxIndices_ = 0;
  int fastShadingMinIndices_ = 0;
  int runtimeMaxDrawIndices_ = 3000000;
  int runtimeMaxDrawIndicesField_ = 1800000;
  std::size_t meshCacheBudgetBytes_ = 512ULL * 1024ULL * 1024ULL;
  std::size_t textureCacheBudgetBytes_ = 256ULL * 1024ULL * 1024ULL;
  std::size_t meshCacheBudgetCount_ = 0;
  std::size_t textureCacheBudgetCount_ = 0;
  ResourceLifetimeManager cacheLifetimeManager_;
  std::unordered_map<std::string, std::size_t> meshCacheBytes_;
  std::unordered_map<std::string, std::size_t> textureCacheBytes_;
  RenderPass renderPass_ = RenderPass::Opaque;
  std::string featureName_ = "cad_opaque";
  std::vector<std::string> dependencies_ = {"geometry_opaque"};
};

}  // namespace repulsor3d
