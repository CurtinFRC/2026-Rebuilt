#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

namespace repulsor3d {

class ISceneAssetResolver;

struct PositionIndexedMesh {
  std::vector<glm::vec3> positions;
  std::vector<std::uint32_t> indices;
};

struct PositionUvIndexedMesh {
  std::vector<glm::vec3> positions;
  std::vector<glm::vec2> uvs;
  std::vector<std::uint32_t> indices;
};

struct PositionNormalMesh {
  struct Vertex {
    glm::vec3 position{0.0F, 0.0F, 0.0F};
    glm::vec3 normal{0.0F, 0.0F, 1.0F};
  };
  std::vector<Vertex> vertices;
};

class IGeometryProvider {
 public:
  virtual ~IGeometryProvider() = default;

  virtual const PositionIndexedMesh& GetUnitCubeMesh() = 0;
  virtual const PositionIndexedMesh& GetUnitSphereMesh() = 0;
  virtual const PositionUvIndexedMesh& GetUnitQuadUvMesh() = 0;
  virtual bool GetCadMesh(const std::string& assetPath, PositionNormalMesh& outMesh) = 0;
};

class DefaultGeometryProvider final : public IGeometryProvider {
 public:
  explicit DefaultGeometryProvider(const ISceneAssetResolver& assetResolver);

  const PositionIndexedMesh& GetUnitCubeMesh() override;
  const PositionIndexedMesh& GetUnitSphereMesh() override;
  const PositionUvIndexedMesh& GetUnitQuadUvMesh() override;
  bool GetCadMesh(const std::string& assetPath, PositionNormalMesh& outMesh) override;

 private:
  static PositionIndexedMesh BuildCubeMesh();
  static PositionIndexedMesh BuildSphereMesh();
  static PositionUvIndexedMesh BuildQuadUvMesh();

  static bool ParseBinaryStl(const std::string& filePath, PositionNormalMesh& outMesh);
  static bool ParseAsciiStl(const std::string& filePath, PositionNormalMesh& outMesh);
  static bool LoadStl(const std::string& filePath, PositionNormalMesh& outMesh);

  static glm::vec3 NormalizeSafe(const glm::vec3& v);
  static glm::vec3 ComputeFallbackNormal(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);

  const ISceneAssetResolver& assetResolver_;

  bool cubeInitialized_ = false;
  bool sphereInitialized_ = false;
  bool quadInitialized_ = false;
  PositionIndexedMesh cubeMesh_;
  PositionIndexedMesh sphereMesh_;
  PositionUvIndexedMesh quadMesh_;
  std::unordered_map<std::string, PositionNormalMesh> cadMeshCache_;
};

}  // namespace repulsor3d
