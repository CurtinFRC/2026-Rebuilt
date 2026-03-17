#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <glm/vec3.hpp>

#include "repulsor3d/render/RenderFeature.hpp"

namespace repulsor3d {

class CadModelRenderFeature final : public IRenderFeature {
 public:
  CadModelRenderFeature() = default;
  ~CadModelRenderFeature() override;

  CadModelRenderFeature(const CadModelRenderFeature&) = delete;
  CadModelRenderFeature& operator=(const CadModelRenderFeature&) = delete;

  bool Initialize(Renderer& renderer) override;
  void Render(const RenderFeatureContext& context, const RendererDrawApi& drawApi) override;

 private:
  struct VertexPN {
    glm::vec3 pos{0.0F, 0.0F, 0.0F};
    glm::vec3 normal{0.0F, 0.0F, 1.0F};
  };

  struct GpuMesh {
    unsigned int vao = 0;
    unsigned int vbo = 0;
    int vertexCount = 0;
  };

  struct CpuMesh {
    std::vector<VertexPN> vertices;
  };

  bool CreateShader();
  static unsigned int CompileShader(unsigned int type, const char* source);
  static bool LinkShader(unsigned int& program, unsigned int vs, unsigned int fs);

  static bool ParseBinaryStl(const std::string& filePath, CpuMesh& outMesh);
  static bool ParseAsciiStl(const std::string& filePath, CpuMesh& outMesh);
  static bool LoadStl(const std::string& filePath, CpuMesh& outMesh);

  static glm::vec3 NormalizeSafe(const glm::vec3& v);
  static glm::vec3 ComputeFallbackNormal(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);

  static bool UploadMesh(const CpuMesh& cpu, GpuMesh& gpu);
  static void DestroyMesh(GpuMesh& gpu);

  static std::string ResolveAssetPath(const std::string& assetPath);
  const GpuMesh* GetOrLoadMesh(const std::string& assetPath);

  unsigned int shader_ = 0;
  int uMvpLoc_ = -1;
  int uModelLoc_ = -1;
  int uColorLoc_ = -1;
  int uLightDirLoc_ = -1;
  bool initialized_ = false;
  std::unordered_map<std::string, GpuMesh> meshCache_;
};

}  // namespace repulsor3d
