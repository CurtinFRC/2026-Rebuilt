#pragma once

#include <memory>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/render/SceneDescriptor.hpp"
#include "repulsor3d/render/SceneFrame.hpp"
#include "repulsor3d/render/SceneModelBuilder.hpp"
#include "repulsor3d/sim/SimWorld.hpp"

namespace repulsor3d {

class IRenderWorldAdapter {
 public:
  virtual ~IRenderWorldAdapter() = default;
  virtual RenderSceneFrame BuildFrame(const ISimWorld& world, const SceneToggleState& toggles) = 0;
};

class SceneModelBuilderRenderWorldAdapter final : public IRenderWorldAdapter {
 public:
  explicit SceneModelBuilderRenderWorldAdapter(std::unique_ptr<ISceneModelBuilder> builder);

  RenderSceneFrame BuildFrame(const ISimWorld& world, const SceneToggleState& toggles) override;

 private:
  std::unique_ptr<ISceneModelBuilder> builder_;
};

class DescriptorDecoratingRenderWorldAdapter final : public IRenderWorldAdapter {
 public:
  DescriptorDecoratingRenderWorldAdapter(std::unique_ptr<IRenderWorldAdapter> inner, SceneDescriptor descriptor);

  RenderSceneFrame BuildFrame(const ISimWorld& world, const SceneToggleState& toggles) override;

 private:
  std::unique_ptr<IRenderWorldAdapter> inner_;
  SceneDescriptor descriptor_;
};

std::unique_ptr<IRenderWorldAdapter> CreateRenderWorldAdapterFromSceneBuilder(std::unique_ptr<ISceneModelBuilder> builder);
std::unique_ptr<IRenderWorldAdapter> CreateDefaultRenderWorldAdapter(const ViewerConfig& cfg);

}  // namespace repulsor3d
