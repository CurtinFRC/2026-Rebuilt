#pragma once

#include <string>
#include <unordered_map>

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
  CadModelRenderFeature() = default;
  ~CadModelRenderFeature() override;

  CadModelRenderFeature(const CadModelRenderFeature&) = delete;
  CadModelRenderFeature& operator=(const CadModelRenderFeature&) = delete;

  std::string Name() const override { return "cad_models"; }
  std::vector<std::string> Dependencies() const override { return {"primitives"}; }

  bool Initialize(Renderer& renderer) override;
  void Render(const RenderFeatureContext& context, const RendererDrawApi& drawApi) override;

 private:
  class FlatLitMaterialPass final : public IMaterialPass {
   public:
    explicit FlatLitMaterialPass(int colorUniformLocation) : colorUniformLocation_(colorUniformLocation) {}
    void Apply(const MaterialInstance& material) override;

   private:
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
    GlVertexArrayHandle vao;
    GlBufferHandle vbo;
    int vertexCount = 0;
  };

  bool CreateShader();
  static unsigned int CompileShader(unsigned int type, const char* source);
  static bool LinkShader(GlProgramHandle& program, unsigned int vs, unsigned int fs);

  static bool UploadMesh(const PositionNormalMesh& cpu, GpuMesh& gpu);
  static void DestroyMesh(GpuMesh& gpu);

  const GpuMesh* GetOrLoadMesh(const std::string& assetPath);

  Renderer* renderer_ = nullptr;
  IGeometryProvider* geometryProvider_ = nullptr;
  IRenderBackend* backend_ = nullptr;

  GlProgramHandle shader_;
  int uMvpLoc_ = -1;
  int uModelLoc_ = -1;
  int uColorLoc_ = -1;
  int uLightDirLoc_ = -1;
  MaterialPipeline materialPipeline_;
  bool initialized_ = false;
  std::unordered_map<std::string, GpuMesh> meshCache_;
};

}  // namespace repulsor3d
