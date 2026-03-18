#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "repulsor3d/render/RenderFeature.hpp"
#include "repulsor3d/render/pipeline/RenderPipelineConfig.hpp"

namespace repulsor3d {

using RenderFeatureFactoryFn = std::function<std::unique_ptr<IRenderFeature>(const RenderPipelinePassSpec& spec)>;

class RenderFeatureFactoryRegistry {
 public:
  void Register(std::string factoryName, RenderFeatureFactoryFn factoryFn);
  std::unique_ptr<IRenderFeature> Create(const RenderPipelinePassSpec& spec) const;
  bool Contains(const std::string& factoryName) const;

 private:
  std::unordered_map<std::string, RenderFeatureFactoryFn> factories_;
};

std::vector<std::unique_ptr<IRenderFeature>> BuildRenderFeaturesFromPipeline(
    const RenderPipelineConfig& config,
    const RenderFeatureFactoryRegistry& registry);

}  // namespace repulsor3d
