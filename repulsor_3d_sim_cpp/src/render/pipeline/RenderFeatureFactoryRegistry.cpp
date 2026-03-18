#include "repulsor3d/render/pipeline/RenderFeatureFactoryRegistry.hpp"

#include <algorithm>
#include <cctype>
#include <utility>

namespace repulsor3d {
namespace {

std::string CanonicalKey(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

}  // namespace

void RenderFeatureFactoryRegistry::Register(std::string factoryName, RenderFeatureFactoryFn factoryFn) {
  if (factoryName.empty() || !factoryFn) {
    return;
  }
  factories_[CanonicalKey(std::move(factoryName))] = std::move(factoryFn);
}

std::unique_ptr<IRenderFeature> RenderFeatureFactoryRegistry::Create(const RenderPipelinePassSpec& spec) const {
  const auto it = factories_.find(CanonicalKey(spec.factory));
  if (it == factories_.end()) {
    return nullptr;
  }
  return it->second(spec);
}

bool RenderFeatureFactoryRegistry::Contains(const std::string& factoryName) const {
  return factories_.contains(CanonicalKey(factoryName));
}

std::vector<std::unique_ptr<IRenderFeature>> BuildRenderFeaturesFromPipeline(
    const RenderPipelineConfig& config,
    const RenderFeatureFactoryRegistry& registry) {
  std::vector<std::unique_ptr<IRenderFeature>> out;
  out.reserve(config.passes.size());

  for (const auto& spec : config.passes) {
    if (!spec.enabled) {
      continue;
    }

    auto feature = registry.Create(spec);
    if (feature != nullptr) {
      out.push_back(std::move(feature));
    }
  }
  return out;
}

}  // namespace repulsor3d
