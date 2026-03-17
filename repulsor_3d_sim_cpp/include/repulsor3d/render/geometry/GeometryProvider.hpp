#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace repulsor3d {

class CadMeshAssetPipeline;
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
    glm::vec4 color{1.0F, 1.0F, 1.0F, 1.0F};
  };
  std::vector<Vertex> vertices;
};

struct CadAssetTelemetryEvent {
  std::string eventName;
  std::string assetPath;
  double milliseconds = 0.0;
};

class IGeometryProvider {
 public:
  virtual ~IGeometryProvider() = default;

  virtual const PositionIndexedMesh& GetUnitCubeMesh() = 0;
  virtual const PositionIndexedMesh& GetUnitSphereMesh() = 0;
  virtual const PositionUvIndexedMesh& GetUnitQuadUvMesh() = 0;
  virtual bool GetCadMesh(const std::string& assetPath, PositionNormalMesh& outMesh) = 0;
  virtual std::vector<CadAssetTelemetryEvent> ConsumeCadAssetTelemetry() = 0;
};

class DefaultGeometryProvider final : public IGeometryProvider {
 public:
  explicit DefaultGeometryProvider(const ISceneAssetResolver& assetResolver);
  ~DefaultGeometryProvider() override;

  const PositionIndexedMesh& GetUnitCubeMesh() override;
  const PositionIndexedMesh& GetUnitSphereMesh() override;
  const PositionUvIndexedMesh& GetUnitQuadUvMesh() override;
  bool GetCadMesh(const std::string& assetPath, PositionNormalMesh& outMesh) override;
  std::vector<CadAssetTelemetryEvent> ConsumeCadAssetTelemetry() override;

 private:
  static PositionIndexedMesh BuildCubeMesh();
  static PositionIndexedMesh BuildSphereMesh();
  static PositionUvIndexedMesh BuildQuadUvMesh();

  const ISceneAssetResolver& assetResolver_;
  std::unique_ptr<CadMeshAssetPipeline> cadAssetPipeline_;

  bool cubeInitialized_ = false;
  bool sphereInitialized_ = false;
  bool quadInitialized_ = false;
  PositionIndexedMesh cubeMesh_;
  PositionIndexedMesh sphereMesh_;
  PositionUvIndexedMesh quadMesh_;
};

}  // namespace repulsor3d
