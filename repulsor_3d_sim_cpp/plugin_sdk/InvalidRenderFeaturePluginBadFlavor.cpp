#include <memory>
#include <string>

#include "repulsor3d/plugins/PluginManifest.hpp"
#include "repulsor3d/plugins/PluginSdk.hpp"
#include "repulsor3d/render/RenderFeaturePlugin.hpp"

#if defined(_WIN32)
#define REPULSOR_RENDER_PLUGIN_EXPORT __declspec(dllexport)
#else
#define REPULSOR_RENDER_PLUGIN_EXPORT __attribute__((visibility("default")))
#endif

namespace repulsor3d {

class InvalidRenderFeaturePlugin final : public IRenderFeaturePlugin {
 public:
  std::string Id() const override { return "invalid_render_bad_flavor"; }
  std::vector<std::unique_ptr<IRenderFeature>> CreateFeatures(const ViewerConfig&) const override {
    return {};
  }
};

}  // namespace repulsor3d

extern "C" REPULSOR_RENDER_PLUGIN_EXPORT int repulsor3d_query_render_feature_plugin_abi_version() {
  return repulsor3d::kRenderFeaturePluginAbiVersion;
}

extern "C" REPULSOR_RENDER_PLUGIN_EXPORT const repulsor3d::PluginSdkInfoV1* repulsor3d_query_plugin_sdk_info_v1() {
  static const repulsor3d::PluginSdkInfoV1 sdkInfo{
      .structSize = static_cast<std::int32_t>(sizeof(repulsor3d::PluginSdkInfoV1)),
      .sdkVersion = repulsor3d::kPluginSdkVersion,
      .minHostSdkVersion = repulsor3d::kPluginSdkVersion,
      .maxHostSdkVersion = repulsor3d::kPluginSdkVersion,
      .sdkFlavor = "invalid_flavor"};
  return &sdkInfo;
}

extern "C" REPULSOR_RENDER_PLUGIN_EXPORT const repulsor3d::PluginManifestV1* repulsor3d_query_plugin_manifest_v1() {
  static const repulsor3d::PluginManifestV1 manifest{
      .structSize = static_cast<std::int32_t>(sizeof(repulsor3d::PluginManifestV1)),
      .pluginKind = static_cast<std::int32_t>(repulsor3d::PluginKind::RenderFeature),
      .abiVersion = repulsor3d::kRenderFeaturePluginAbiVersion,
      .minHostAbiVersion = repulsor3d::kRenderFeaturePluginAbiVersion,
      .maxHostAbiVersion = repulsor3d::kRenderFeaturePluginAbiVersion,
      .pluginId = "invalid_render_bad_flavor",
      .pluginVersion = "0.0.1",
      .buildSignature = "invalid-bad-flavor",
      .capabilityFlags = 0ULL};
  return &manifest;
}

extern "C" REPULSOR_RENDER_PLUGIN_EXPORT repulsor3d::IRenderFeaturePlugin* repulsor3d_create_render_feature_plugin() {
  return new repulsor3d::InvalidRenderFeaturePlugin();
}

extern "C" REPULSOR_RENDER_PLUGIN_EXPORT void repulsor3d_destroy_render_feature_plugin(
    repulsor3d::IRenderFeaturePlugin* plugin) {
  delete plugin;
}

