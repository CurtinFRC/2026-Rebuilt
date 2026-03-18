#include "repulsor3d/datasource/DataSourcePluginAbi.hpp"
#include "repulsor3d/plugins/PluginManifest.hpp"

#if defined(_WIN32)
#define REPULSOR_DS_PLUGIN_EXPORT __declspec(dllexport)
#else
#define REPULSOR_DS_PLUGIN_EXPORT
#endif

namespace repulsor3d {

class GenericDataSourcePluginTemplate final : public ISnapshotSource {
 public:
  explicit GenericDataSourcePluginTemplate(const ViewerConfig& cfg) : cfg_(cfg) {}

  SnapshotBundle Read() override {
    SnapshotBundle bundle;
    bundle.connected = true;
    bundle.method = "plugin_template";
    bundle.snapshot.pose = Pose2D{0.0, 0.0, 0.0};
    (void)cfg_;
    return bundle;
  }

 private:
  ViewerConfig cfg_;
};

}  // namespace repulsor3d

extern "C" REPULSOR_DS_PLUGIN_EXPORT int repulsor3d_query_data_source_plugin_abi_version() {
  return repulsor3d::kDataSourcePluginAbiVersion;
}

extern "C" REPULSOR_DS_PLUGIN_EXPORT const repulsor3d::PluginManifestV1* repulsor3d_query_plugin_manifest_v1() {
  static const repulsor3d::PluginManifestV1 manifest{
      .structSize = static_cast<std::int32_t>(sizeof(repulsor3d::PluginManifestV1)),
      .pluginKind = static_cast<std::int32_t>(repulsor3d::PluginKind::DataSource),
      .abiVersion = repulsor3d::kDataSourcePluginAbiVersion,
      .minHostAbiVersion = repulsor3d::kDataSourcePluginAbiVersion,
      .maxHostAbiVersion = repulsor3d::kDataSourcePluginAbiVersion,
      .pluginId = "generic_datasource_plugin",
      .pluginVersion = "1.1.0",
      .buildSignature = "generic-datasource-template",
      .capabilityFlags = repulsor3d::ToCapabilityBits(repulsor3d::PluginCapability::DiagnosticsAware)};
  return &manifest;
}

extern "C" REPULSOR_DS_PLUGIN_EXPORT repulsor3d::ISnapshotSource* repulsor3d_create_data_source_plugin(
    const repulsor3d::ViewerConfig& cfg) {
  return new repulsor3d::GenericDataSourcePluginTemplate(cfg);
}

extern "C" REPULSOR_DS_PLUGIN_EXPORT void repulsor3d_destroy_data_source_plugin(repulsor3d::ISnapshotSource* source) {
  delete source;
}
