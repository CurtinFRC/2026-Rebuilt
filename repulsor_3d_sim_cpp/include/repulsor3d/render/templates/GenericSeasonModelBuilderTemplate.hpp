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

    // TODO: Map bundle.snapshot to entities/primitives for your season:
    // frame.entities.push_back({.id = "robot", .pass = RenderPass::Opaque, .payload = MeshInstancePrimitive{...}});
    // frame.entities.push_back({.id = "fov", .pass = RenderPass::Transparent, .payload = LinePrimitive{...}});
    // frame.spheres.push_back(...);  // legacy path still supported

    frame.overlayLines.push_back({"[Template] scene profile active", {0.92F, 0.92F, 0.92F, 0.90F}, OverlayAnchor::TopLeft});
    frame.overlayLines.push_back(
        {"Pieces: " + std::to_string(bundle.pieces), {0.92F, 0.92F, 0.92F, 0.90F}, OverlayAnchor::TopLeft});
    frame.overlayLines.push_back({"Method: " + bundle.method, {0.80F, 0.90F, 1.00F, 0.90F}, OverlayAnchor::TopRight});

    return frame;
  }

 private:
  ViewerConfig cfg_;
};

}  // namespace repulsor3d
