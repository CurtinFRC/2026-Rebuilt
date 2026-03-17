#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "repulsor3d/render/geometry/GeometryProvider.hpp"

namespace repulsor3d {

class ISceneAssetResolver;

class ICadMeshImporter {
 public:
  virtual ~ICadMeshImporter() = default;
  virtual bool Import(const std::string& resolvedAssetPath, PositionNormalMesh& outMesh) = 0;
};

class StlCadMeshImporter final : public ICadMeshImporter {
 public:
  bool Import(const std::string& resolvedAssetPath, PositionNormalMesh& outMesh) override;

 private:
  static bool ParseBinaryStl(const std::string& filePath, PositionNormalMesh& outMesh);
  static bool ParseAsciiStl(const std::string& filePath, PositionNormalMesh& outMesh);
};

class ObjCadMeshImporter final : public ICadMeshImporter {
 public:
  bool Import(const std::string& resolvedAssetPath, PositionNormalMesh& outMesh) override;
};

class GltfCadMeshImporter final : public ICadMeshImporter {
 public:
  bool Import(const std::string& resolvedAssetPath, PositionNormalMesh& outMesh) override;
};

class MultiFormatCadMeshImporter final : public ICadMeshImporter {
 public:
  bool Import(const std::string& resolvedAssetPath, PositionNormalMesh& outMesh) override;

 private:
  StlCadMeshImporter stlImporter_;
  ObjCadMeshImporter objImporter_;
  GltfCadMeshImporter gltfImporter_;
};

class ICadMeshCooker {
 public:
  virtual ~ICadMeshCooker() = default;
  virtual void Cook(PositionNormalMesh& mesh) = 0;
};

class PassThroughCadMeshCooker final : public ICadMeshCooker {
 public:
  void Cook(PositionNormalMesh& mesh) override;
};

class ICadMeshCache {
 public:
  virtual ~ICadMeshCache() = default;
  virtual bool TryLoad(const std::string& key, PositionNormalMesh& outMesh) const = 0;
  virtual void Store(const std::string& key, const PositionNormalMesh& mesh) = 0;
};

class InMemoryCadMeshCache final : public ICadMeshCache {
 public:
  bool TryLoad(const std::string& key, PositionNormalMesh& outMesh) const override;
  void Store(const std::string& key, const PositionNormalMesh& mesh) override;

 private:
  std::unordered_map<std::string, PositionNormalMesh> cache_;
};

class FileCadMeshCache final : public ICadMeshCache {
 public:
  explicit FileCadMeshCache(std::string rootDirectory);

  bool TryLoad(const std::string& key, PositionNormalMesh& outMesh) const override;
  void Store(const std::string& key, const PositionNormalMesh& mesh) override;

 private:
  std::string KeyToPath(const std::string& key) const;
  std::string rootDirectory_;
};

class CompositeCadMeshCache final : public ICadMeshCache {
 public:
  CompositeCadMeshCache(std::unique_ptr<ICadMeshCache> first, std::unique_ptr<ICadMeshCache> second);

  bool TryLoad(const std::string& key, PositionNormalMesh& outMesh) const override;
  void Store(const std::string& key, const PositionNormalMesh& mesh) override;

 private:
  std::unique_ptr<ICadMeshCache> first_;
  std::unique_ptr<ICadMeshCache> second_;
};

std::unique_ptr<ICadMeshImporter> CreateDefaultCadMeshImporter();
std::unique_ptr<ICadMeshCache> CreateDefaultCadMeshCache();

class CadMeshAssetPipeline {
 public:
  struct TelemetryEvent {
    std::string eventName;
    std::string assetPath;
    double milliseconds = 0.0;
  };

  explicit CadMeshAssetPipeline(
      const ISceneAssetResolver& resolver,
      std::unique_ptr<ICadMeshImporter> importer = CreateDefaultCadMeshImporter(),
      std::unique_ptr<ICadMeshCooker> cooker = std::make_unique<PassThroughCadMeshCooker>(),
      std::unique_ptr<ICadMeshCache> cache = CreateDefaultCadMeshCache());

  bool Load(const std::string& assetPath, PositionNormalMesh& outMesh);
  std::vector<TelemetryEvent> ConsumeTelemetryEvents();

 private:
  const ISceneAssetResolver& resolver_;
  std::unique_ptr<ICadMeshImporter> importer_;
  std::unique_ptr<ICadMeshCooker> cooker_;
  std::unique_ptr<ICadMeshCache> cache_;
  mutable std::mutex telemetryMutex_;
  std::vector<TelemetryEvent> telemetryEvents_;
};

}  // namespace repulsor3d
