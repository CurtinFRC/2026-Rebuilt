#include <memory>
#include <string>

#include "repulsor3d/modules/SeasonModule.hpp"
#include "repulsor3d/plugins/PluginManifest.hpp"

namespace repulsor3d {

class InvalidSeasonNoSdkAdapter final : public IRenderWorldAdapter {
 public:
  RenderSceneFrame BuildFrame(const ISimWorld& world, const SceneToggleState& toggles) override {
    RenderSceneFrame frame;
    frame.drawFieldImage = toggles.showFieldImage;
    frame.overlayLines.push_back({"invalid season plugin (no sdk symbol)"});
    frame.overlayLines.push_back({"pieces=" + std::to_string(world.PieceCount())});
    return frame;
  }
};

class InvalidSeasonNoSdkModule final : public ISeasonModule {
 public:
  std::string Id() const override { return "invalid_season_no_sdk"; }
  std::unique_ptr<IRenderWorldAdapter> CreateWorldAdapter(const ViewerConfig&) const override {
    return std::make_unique<InvalidSeasonNoSdkAdapter>();
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
      .pluginId = "invalid_season_no_sdk",
      .pluginVersion = "0.0.1",
      .buildSignature = "invalid-no-sdk",
      .capabilityFlags = 0ULL};
  return &manifest;
}

extern "C" REPULSOR_PLUGIN_EXPORT repulsor3d::ISeasonModule* repulsor3d_create_season_module() {
  return new repulsor3d::InvalidSeasonNoSdkModule();
}

extern "C" REPULSOR_PLUGIN_EXPORT void repulsor3d_destroy_season_module(repulsor3d::ISeasonModule* module) {
  delete module;
}

