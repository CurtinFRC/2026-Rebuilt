#pragma once

#include <memory>

#include "repulsor3d/Config.hpp"

namespace repulsor3d {

class ISeasonModule;
class IRenderWorldAdapter;

namespace app::composition {

struct SeasonWorldCompositionResult {
  std::unique_ptr<ISeasonModule> module;
  std::unique_ptr<IRenderWorldAdapter> adapter;
};

SeasonWorldCompositionResult ComposeSeasonWorldAdapter(const ViewerConfig& cfg);

}  // namespace app::composition
}  // namespace repulsor3d

