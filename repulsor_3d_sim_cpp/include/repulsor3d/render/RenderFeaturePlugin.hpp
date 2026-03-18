#pragma once

#include <memory>
#include <string>
#include <vector>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/render/RenderFeature.hpp"

namespace repulsor3d {

inline constexpr int kRenderFeaturePluginAbiVersion = 1;

class IRenderFeaturePlugin {
 public:
  virtual ~IRenderFeaturePlugin() = default;
  virtual std::string Id() const = 0;
  virtual std::vector<std::unique_ptr<IRenderFeature>> CreateFeatures(const ViewerConfig& cfg) const = 0;
};

using CreateRenderFeaturePluginAbiFn = IRenderFeaturePlugin* (*)();
using DestroyRenderFeaturePluginAbiFn = void (*)(IRenderFeaturePlugin*);
using QueryRenderFeaturePluginAbiVersionFn = int (*)();

std::unique_ptr<IRenderFeaturePlugin> CreateRenderFeaturePluginFromPath(const std::string& pluginPath);

}  // namespace repulsor3d

