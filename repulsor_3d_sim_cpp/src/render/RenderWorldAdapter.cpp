#include "repulsor3d/render/RenderWorldAdapter.hpp"

#include <utility>

#include "repulsor3d/modules/SeasonModule.hpp"
#include "repulsor3d/render/SceneModelBuilderFactory.hpp"
#include "repulsor3d/render/templates/GenericSeasonModelBuilderTemplate.hpp"

namespace repulsor3d {

SceneModelBuilderRenderWorldAdapter::SceneModelBuilderRenderWorldAdapter(std::unique_ptr<ISceneModelBuilder> builder)
    : builder_(std::move(builder)) {}

RenderSceneFrame SceneModelBuilderRenderWorldAdapter::BuildFrame(const ISimWorld& world, const SceneToggleState& toggles) {
  if (builder_ == nullptr) {
    return {};
  }
  return builder_->BuildFrame(world.AsSnapshotBundle(), toggles);
}

DescriptorDecoratingRenderWorldAdapter::DescriptorDecoratingRenderWorldAdapter(
    std::unique_ptr<IRenderWorldAdapter> inner,
    SceneDescriptor descriptor)
    : inner_(std::move(inner)), descriptor_(std::move(descriptor)) {}

RenderSceneFrame DescriptorDecoratingRenderWorldAdapter::BuildFrame(const ISimWorld& world, const SceneToggleState& toggles) {
  RenderSceneFrame frame;
  if (inner_ != nullptr) {
    frame = inner_->BuildFrame(world, toggles);
  }

  if (descriptor_.drawFieldImage.has_value()) {
    frame.drawFieldImage = *descriptor_.drawFieldImage;
  }
  if (descriptor_.drawGrid.has_value()) {
    frame.drawGrid = *descriptor_.drawGrid;
  }
  if (descriptor_.drawAxes.has_value()) {
    frame.drawAxes = *descriptor_.drawAxes;
  }

  frame.spheres.insert(frame.spheres.end(), descriptor_.staticSpheres.begin(), descriptor_.staticSpheres.end());
  frame.boxes.insert(frame.boxes.end(), descriptor_.staticBoxes.begin(), descriptor_.staticBoxes.end());
  frame.lines.insert(frame.lines.end(), descriptor_.staticLines.begin(), descriptor_.staticLines.end());
  frame.meshInstances.insert(frame.meshInstances.end(), descriptor_.staticMeshes.begin(), descriptor_.staticMeshes.end());
  frame.entities.insert(frame.entities.end(), descriptor_.staticEntities.begin(), descriptor_.staticEntities.end());

  frame.overlayLines.insert(
      frame.overlayLines.end(),
      descriptor_.staticOverlayLines.begin(),
      descriptor_.staticOverlayLines.end());
  return frame;
}

std::unique_ptr<IRenderWorldAdapter> CreateRenderWorldAdapterFromSceneBuilder(std::unique_ptr<ISceneModelBuilder> builder) {
  if (builder == nullptr) {
    return std::make_unique<SceneModelBuilderRenderWorldAdapter>(
        std::make_unique<GenericSeasonModelBuilderTemplate>(ViewerConfig{}));
  }
  return std::make_unique<SceneModelBuilderRenderWorldAdapter>(std::move(builder));
}

std::unique_ptr<IRenderWorldAdapter> CreateDefaultRenderWorldAdapter(const ViewerConfig& cfg) {
  if (auto seasonModule = CreateDefaultSeasonModule(cfg); seasonModule != nullptr) {
    if (auto adapter = seasonModule->CreateWorldAdapter(cfg); adapter != nullptr) {
      return adapter;
    }
  }
  return CreateRenderWorldAdapterFromSceneBuilder(CreateDefaultSceneModelBuilder(cfg));
}

}  // namespace repulsor3d
