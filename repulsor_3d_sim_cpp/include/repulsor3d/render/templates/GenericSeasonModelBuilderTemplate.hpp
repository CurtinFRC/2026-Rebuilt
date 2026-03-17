#pragma once

#include <string>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/render/SceneModelBuilder.hpp"

namespace repulsor3d {

// Template scaffold for a future season implementation.
// Copy this file, rename the class, and implement BuildFrame().
class GenericSeasonModelBuilderTemplate final : public ISceneModelBuilder {
 public:
  explicit GenericSeasonModelBuilderTemplate(const ViewerConfig& cfg) : cfg_(cfg) {}

  RenderSceneFrame BuildFrame(const SnapshotBundle& bundle, const SceneToggleState& toggles) override {
    RenderSceneFrame frame;

    frame.drawFieldImage = toggles.showFieldImage;
    frame.drawGrid = true;
    frame.drawAxes = true;

    // TODO: Map bundle.snapshot to generic primitives.
    // frame.spheres.push_back(...)
    // frame.boxes.push_back(...)
    // frame.lines.push_back(...)

    frame.overlayLines.push_back({"[Template] scene profile active"});
    frame.overlayLines.push_back({"Pieces: " + std::to_string(bundle.pieces)});

    return frame;
  }

 private:
  ViewerConfig cfg_;
};

}  // namespace repulsor3d
