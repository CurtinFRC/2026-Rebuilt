#pragma once

#include <string>
#include <unordered_map>

#include <glm/vec3.hpp>

#include "repulsor3d/render/RenderFeature.hpp"
#include "repulsor3d/render/geometry/GeometryProvider.hpp"
#include "repulsor3d/render/material/Material.hpp"

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

  struct VertexPN {
    glm::vec3 pos{0.0F, 0.0F, 0.0F};
    glm::vec3 normal{0.0F, 0.0F, 1.0F};
  };

  struct GpuMesh {
    unsigned int vao = 0;
    unsigned int vbo = 0;
    int vertexCount = 0;
  };

  bool CreateShader();
  static unsigned int CompileShader(unsigned int type, const char* source);
  static bool LinkShader(unsigned int& program, unsigned int vs, unsigned int fs);

  static bool UploadMesh(const PositionNormalMesh& cpu, GpuMesh& gpu);
  static void DestroyMesh(GpuMesh& gpu);

  const GpuMesh* GetOrLoadMesh(const std::string& assetPath);

  Renderer* renderer_ = nullptr;
  IGeometryProvider* geometryProvider_ = nullptr;

  unsigned int shader_ = 0;
  int uMvpLoc_ = -1;
  int uModelLoc_ = -1;
  int uColorLoc_ = -1;
  int uLightDirLoc_ = -1;
  FlatLitMaterialPass materialPass_{-1};
  bool initialized_ = false;
  std::unordered_map<std::string, GpuMesh> meshCache_;
};

}  // namespace repulsor3d
