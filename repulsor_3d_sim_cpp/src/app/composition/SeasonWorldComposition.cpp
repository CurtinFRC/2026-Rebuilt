#include "repulsor3d/app/composition/SeasonWorldComposition.hpp"

#include <utility>

#include "repulsor3d/modules/SeasonModule.hpp"
#include "repulsor3d/render/RenderWorldAdapter.hpp"
#include "repulsor3d/render/SceneModelBuilderFactory.hpp"

namespace repulsor3d::app::composition {

SeasonWorldCompositionResult ComposeSeasonWorldAdapter(const ViewerConfig& cfg) {
  SeasonWorldCompositionResult out;

  if (!cfg.seasonModulePluginPath.empty()) {
    out.module = CreateSeasonModuleFromPlugin(cfg.seasonModulePluginPath);
  }
  if (out.module == nullptr) {
    out.module = CreateSeasonModule(cfg.sceneProfile);
  }
  if (out.module == nullptr) {
    out.module = CreateSeasonModule("default");
  }

  if (out.module != nullptr) {
    out.adapter = out.module->CreateWorldAdapter(cfg);
  }
  if (out.adapter == nullptr) {
    out.adapter = CreateRenderWorldAdapterFromSceneBuilder(CreateDefaultSceneModelBuilder(cfg));
  }
  return out;
}

}  // namespace repulsor3d::app::composition

