#include "repulsor3d/datasource/DataSourcePluginAbi.hpp"

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

extern "C" REPULSOR_DS_PLUGIN_EXPORT repulsor3d::ISnapshotSource* repulsor3d_create_data_source_plugin(
    const repulsor3d::ViewerConfig& cfg) {
  return new repulsor3d::GenericDataSourcePluginTemplate(cfg);
}

extern "C" REPULSOR_DS_PLUGIN_EXPORT void repulsor3d_destroy_data_source_plugin(repulsor3d::ISnapshotSource* source) {
  delete source;
}
