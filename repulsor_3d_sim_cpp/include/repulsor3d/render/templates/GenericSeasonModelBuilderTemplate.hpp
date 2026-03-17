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
    // frame.entities.push_back({
    //   .id = "robot_pose",
    //   .pass = RenderPass::Opaque,
    //   .payload = BoxPrimitive{.center = {0.0F, 0.0F, 0.2F}, .size = {0.8F, 0.8F, 0.4F}},
    //   .transform = Transform3D{.position = {2.0F, 1.0F, 0.0F}, .rotationDeg = {0.0F, 0.0F, 45.0F}, .scale = {1.0F, 1.0F, 1.0F}},
    //   .hasTransform = true,
    //   .culling = EntityCulling{.enabled = true, .boundsRadius = 0.9F}
    // });
    // frame.entities.push_back({
    //   .id = "robot_mesh",
    //   .pass = RenderPass::Opaque,
    //   .payload = MeshInstancePrimitive{.assetPath = "models/robot.glb", .position = {0.0F, 0.0F, 0.0F}},
    //   .parentId = "robot_pose"
    // });
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
