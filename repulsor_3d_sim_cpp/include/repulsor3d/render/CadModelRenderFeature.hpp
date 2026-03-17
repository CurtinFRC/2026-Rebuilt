#pragma once

#include <cstdint>
#include <cstddef>
#include <future>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include "repulsor3d/render/RenderFeature.hpp"
#include "repulsor3d/render/backend/GlHandles.hpp"
#include "repulsor3d/render/backend/RenderBackend.hpp"
#include "repulsor3d/render/geometry/GeometryProvider.hpp"
#include "repulsor3d/render/material/Material.hpp"
#include "repulsor3d/render/material/MaterialPipeline.hpp"

namespace repulsor3d {

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
    struct LodGpu {
      GlVertexArrayHandle vao;
      GlBufferHandle vbo;
      GlBufferHandle ebo;
      int vertexCount = 0;
      int indexCount = 0;
    };
    std::vector<LodGpu> lods;
    glm::vec3 boundsCenter{0.0F, 0.0F, 0.0F};
    float boundsRadius = 1.0F;
  };

  struct PreparedCpuMesh {
    struct LodCpu {
      std::vector<PositionNormalMesh::Vertex> vertices;
      std::vector<std::uint32_t> indices;
    };
    std::vector<LodCpu> lods;
    glm::vec3 boundsCenter{0.0F, 0.0F, 0.0F};
    float boundsRadius = 1.0F;
  };

  struct PendingLoad {
    std::future<PreparedCpuMesh> future;
  };

  bool CreateShader();
  static unsigned int CompileShader(unsigned int type, const char* source);
  static bool LinkShader(GlProgramHandle& program, unsigned int vs, unsigned int fs);

  static PreparedCpuMesh PrepareCpuMesh(const PositionNormalMesh& cpu);
  static bool UploadMesh(const PreparedCpuMesh& cpu, GpuMesh& gpu, IRenderBackend& backend);
  static void DestroyMesh(GpuMesh& gpu);
  static std::size_t SelectLodLevel(const GpuMesh& mesh, const glm::mat4& mvp, int viewportWidth, int viewportHeight);

  const GpuMesh* GetOrLoadMesh(const std::string& assetPath);

  Renderer* renderer_ = nullptr;
  IGeometryProvider* geometryProvider_ = nullptr;
  IRenderBackend* backend_ = nullptr;

  GlProgramHandle shader_;
  int uMvpLoc_ = -1;
  int uNormalMatrixLoc_ = -1;
  int uColorLoc_ = -1;
  int uLightDirLoc_ = -1;
  int uUseAssetColorLoc_ = -1;
  MaterialPipeline materialPipeline_;
  bool initialized_ = false;
  std::unordered_map<std::string, GpuMesh> meshCache_;
  std::unordered_map<std::string, PendingLoad> pendingLoads_;
  std::unordered_set<std::string> failedLoads_;
  RenderPass renderPass_ = RenderPass::Opaque;
  std::string featureName_ = "cad_opaque";
  std::vector<std::string> dependencies_ = {"geometry_opaque"};
};

}  // namespace repulsor3d
