#include <memory>
#include <string>

#include "repulsor3d/modules/SeasonModule.hpp"
#include "repulsor3d/plugins/PluginManifest.hpp"

namespace repulsor3d {

class GenericSdkWorldAdapter final : public IRenderWorldAdapter {
 public:
  RenderSceneFrame BuildFrame(const ISimWorld& world, const SceneToggleState& toggles) override {
    RenderSceneFrame frame;
    frame.drawFieldImage = toggles.showFieldImage;
    frame.drawGrid = true;
    frame.drawAxes = true;

    frame.overlayLines.push_back({"[PluginSDK] Dynamic season module loaded"});
    frame.overlayLines.push_back({"Pieces: " + std::to_string(world.PieceCount())});
    frame.overlayLines.push_back({"Method: " + world.Method()});
    return frame;
  }
};

class GenericSdkSeasonModule final : public ISeasonModule {
 public:
  std::string Id() const override { return "generic_sdk_plugin"; }

  std::unique_ptr<IRenderWorldAdapter> CreateWorldAdapter(const ViewerConfig& /*cfg*/) const override {
    return std::make_unique<GenericSdkWorldAdapter>();
  }
};

}  // namespace repulsor3d

#if defined(_WIN32)
#define REPULSOR_PLUGIN_EXPORT __declspec(dllexport)
#else
#define REPULSOR_PLUGIN_EXPORT __attribute__((visibility("default")))
#endif

extern "C" REPULSOR_PLUGIN_EXPORT int repulsor3d_query_season_module_abi_version() {
  return repulsor3d::kSeasonModuleAbiVersion;
}

extern "C" REPULSOR_PLUGIN_EXPORT const repulsor3d::PluginManifestV1* repulsor3d_query_plugin_manifest_v1() {
  static const repulsor3d::PluginManifestV1 manifest{
      .structSize = static_cast<std::int32_t>(sizeof(repulsor3d::PluginManifestV1)),
      .pluginKind = static_cast<std::int32_t>(repulsor3d::PluginKind::SeasonModule),
      .abiVersion = repulsor3d::kSeasonModuleAbiVersion,
      .minHostAbiVersion = repulsor3d::kSeasonModuleAbiVersion,
      .maxHostAbiVersion = repulsor3d::kSeasonModuleAbiVersion,
      .pluginId = "generic_sdk_plugin",
      .pluginVersion = "1.0.0"};
  return &manifest;
}

extern "C" REPULSOR_PLUGIN_EXPORT repulsor3d::ISeasonModule* repulsor3d_create_season_module() {
  return new repulsor3d::GenericSdkSeasonModule();
}

extern "C" REPULSOR_PLUGIN_EXPORT void repulsor3d_destroy_season_module(repulsor3d::ISeasonModule* module) {
  delete module;
}
