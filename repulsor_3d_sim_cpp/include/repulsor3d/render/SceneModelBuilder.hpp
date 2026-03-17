#pragma once

#include "repulsor3d/Config.hpp"
#include "repulsor3d/Model.hpp"
#include "repulsor3d/render/SceneFrame.hpp"

namespace repulsor3d {

struct SceneToggleState {
  bool showCameraDebug = true;
  bool showTruthFuel = true;
  bool showAgeFilteredFuel = false;
  bool showFieldImage = true;
};

class ISceneModelBuilder {
 public:
  virtual ~ISceneModelBuilder() = default;
  virtual RenderSceneFrame BuildFrame(const SnapshotBundle& bundle, const SceneToggleState& toggles) = 0;
};

}  // namespace repulsor3d
