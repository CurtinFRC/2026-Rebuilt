#pragma once

#include <memory>
#include <string>

#include "repulsor3d/modules/SeasonModule.hpp"
#include "repulsor3d/render/RenderWorldAdapter.hpp"
#include "repulsor3d/render/templates/GenericSeasonModelBuilderTemplate.hpp"

namespace repulsor3d {

// Copy this file into your module project and rename the class.
class GenericSeasonModuleTemplate final : public ISeasonModule {
 public:
  std::string Id() const override { return "genericseason"; }

  std::unique_ptr<IRenderWorldAdapter> CreateWorldAdapter(const ViewerConfig& cfg) const override {
    return CreateRenderWorldAdapterFromSceneBuilder(std::make_unique<GenericSeasonModelBuilderTemplate>(cfg));
  }
};

// Plugin ABI exports (for dynamic module .dll/.so):
// extern "C" __declspec(dllexport) int repulsor3d_query_season_module_abi_version() {
//   return repulsor3d::kSeasonModuleAbiVersion;
// }
// extern "C" __declspec(dllexport) repulsor3d::ISeasonModule* repulsor3d_create_season_module() {
//   return new GenericSeasonModuleTemplate();
// }
// extern "C" __declspec(dllexport) void repulsor3d_destroy_season_module(repulsor3d::ISeasonModule* module) {
//   delete module;
// }

}  // namespace repulsor3d
