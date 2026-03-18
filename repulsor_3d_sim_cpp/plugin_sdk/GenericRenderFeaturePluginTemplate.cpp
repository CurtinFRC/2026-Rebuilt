#include "repulsor3d/render/RenderFeaturePlugin.hpp"

namespace repulsor3d {
namespace {

class PluginBannerFeature final : public IRenderFeature {
 public:
  std::string Name() const override { return "plugin_banner_overlay"; }
  std::vector<std::string> Dependencies() const override { return {"overlay"}; }

  void Render(const RenderFeatureContext& /*context*/, const RendererDrawApi& /*drawApi*/) override {
    // Intentionally no-op in SDK template: host project decides where/how to present plugin UI.
  }
};

class GenericRenderFeaturePluginTemplate final : public IRenderFeaturePlugin {
 public:
  std::string Id() const override { return "generic_render_feature_plugin"; }

  std::vector<std::unique_ptr<IRenderFeature>> CreateFeatures(const ViewerConfig& /*cfg*/) const override {
    std::vector<std::unique_ptr<IRenderFeature>> features;
    features.push_back(std::make_unique<PluginBannerFeature>());
    return features;
  }
};

}  // namespace
}  // namespace repulsor3d

#if defined(_WIN32)
#define REPULSOR_RENDER_PLUGIN_EXPORT __declspec(dllexport)
#else
#define REPULSOR_RENDER_PLUGIN_EXPORT __attribute__((visibility("default")))
#endif

extern "C" REPULSOR_RENDER_PLUGIN_EXPORT int repulsor3d_query_render_feature_plugin_abi_version() {
  return repulsor3d::kRenderFeaturePluginAbiVersion;
}

extern "C" REPULSOR_RENDER_PLUGIN_EXPORT repulsor3d::IRenderFeaturePlugin* repulsor3d_create_render_feature_plugin() {
  return new repulsor3d::GenericRenderFeaturePluginTemplate();
}

extern "C" REPULSOR_RENDER_PLUGIN_EXPORT void repulsor3d_destroy_render_feature_plugin(
    repulsor3d::IRenderFeaturePlugin* plugin) {
  delete plugin;
}
