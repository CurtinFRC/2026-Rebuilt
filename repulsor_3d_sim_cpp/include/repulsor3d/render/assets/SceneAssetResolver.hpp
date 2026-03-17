#pragma once

#include <string>

namespace repulsor3d {

class ISceneAssetResolver {
 public:
  virtual ~ISceneAssetResolver() = default;
  virtual std::string ResolveFilePath(const std::string& rawPath) const = 0;
};

class DefaultSceneAssetResolver final : public ISceneAssetResolver {
 public:
  std::string ResolveFilePath(const std::string& rawPath) const override;
};

}  // namespace repulsor3d
