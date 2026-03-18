#include "repulsor3d/datasource/DataSourcePluginAbi.hpp"
#include "repulsor3d/plugins/PluginManifest.hpp"
#include "repulsor3d/plugins/PluginSdk.hpp"

#if defined(_WIN32)
#define REPULSOR_DS_PLUGIN_EXPORT __declspec(dllexport)
#else
#define REPULSOR_DS_PLUGIN_EXPORT
#endif

namespace repulsor3d {

class InvalidDataSourceBadFlavor final : public ISnapshotSource {
 public:
  explicit InvalidDataSourceBadFlavor(const ViewerConfig&) {}

  SnapshotBundle Read() override {
    SnapshotBundle bundle;
    bundle.connected = true;
    bundle.method = "invalid_bad_flavor";
    return bundle;
  }
};

}  // namespace repulsor3d

extern "C" REPULSOR_DS_PLUGIN_EXPORT int repulsor3d_query_data_source_plugin_abi_version() {
  return repulsor3d::kDataSourcePluginAbiVersion;
}

extern "C" REPULSOR_DS_PLUGIN_EXPORT const repulsor3d::PluginSdkInfoV1* repulsor3d_query_plugin_sdk_info_v1() {
  static const repulsor3d::PluginSdkInfoV1 sdkInfo{
      .structSize = static_cast<std::int32_t>(sizeof(repulsor3d::PluginSdkInfoV1)),
      .sdkVersion = repulsor3d::kPluginSdkVersion,
      .minHostSdkVersion = repulsor3d::kPluginSdkVersion,
      .maxHostSdkVersion = repulsor3d::kPluginSdkVersion,
      .sdkFlavor = "invalid_flavor"};
  return &sdkInfo;
}

extern "C" REPULSOR_DS_PLUGIN_EXPORT const repulsor3d::PluginManifestV1* repulsor3d_query_plugin_manifest_v1() {
  static const repulsor3d::PluginManifestV1 manifest{
      .structSize = static_cast<std::int32_t>(sizeof(repulsor3d::PluginManifestV1)),
      .pluginKind = static_cast<std::int32_t>(repulsor3d::PluginKind::DataSource),
      .abiVersion = repulsor3d::kDataSourcePluginAbiVersion,
      .minHostAbiVersion = repulsor3d::kDataSourcePluginAbiVersion,
      .maxHostAbiVersion = repulsor3d::kDataSourcePluginAbiVersion,
      .pluginId = "invalid_datasource_bad_flavor",
      .pluginVersion = "0.0.1",
      .buildSignature = "invalid-bad-flavor",
      .capabilityFlags = 0ULL};
  return &manifest;
}

extern "C" REPULSOR_DS_PLUGIN_EXPORT repulsor3d::ISnapshotSource* repulsor3d_create_data_source_plugin(
    const repulsor3d::ViewerConfig& cfg) {
  return new repulsor3d::InvalidDataSourceBadFlavor(cfg);
}

extern "C" REPULSOR_DS_PLUGIN_EXPORT void repulsor3d_destroy_data_source_plugin(repulsor3d::ISnapshotSource* source) {
  delete source;
}

