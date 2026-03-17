#include "repulsor3d/render/RenderWorldAdapter.hpp"

#include <filesystem>
#include <utility>

#include "repulsor3d/modules/SeasonModule.hpp"
#include "repulsor3d/render/SceneDescriptor.hpp"
#include "repulsor3d/render/SceneModelBuilderFactory.hpp"
#include "repulsor3d/render/templates/GenericSeasonModelBuilderTemplate.hpp"

namespace repulsor3d {
namespace {

class HotReloadingRenderWorldAdapter final : public IRenderWorldAdapter {
 public:
  explicit HotReloadingRenderWorldAdapter(ViewerConfig cfg) : cfg_(std::move(cfg)) {}

  RenderSceneFrame BuildFrame(const ISimWorld& world, const SceneToggleState& toggles) override {
    MaybeRebuildIfNeeded();
    if (adapter_ == nullptr) {
      return {};
    }
    return adapter_->BuildFrame(world, toggles);
  }

 private:
  static bool QueryFileWriteTime(const std::string& path, std::filesystem::file_time_type& outWriteTime) {
    if (path.empty()) {
      return false;
    }
    std::error_code ec;
    if (!std::filesystem::exists(path, ec) || ec) {
      return false;
    }
    outWriteTime = std::filesystem::last_write_time(path, ec);
    return !ec;
  }

  bool DescriptorChanged(std::string& outResolvedPath, std::filesystem::file_time_type& outWriteTime) const {
    outResolvedPath = ResolveSceneDescriptorPathForProfile(cfg_);
    const bool exists = QueryFileWriteTime(outResolvedPath, outWriteTime);

    if (!initialized_) {
      return true;
    }
    if (outResolvedPath != descriptorPath_) {
      return true;
    }
    if (exists != descriptorWriteTimeKnown_) {
      return true;
    }
    if (exists && outWriteTime != descriptorWriteTime_) {
      return true;
    }
    return false;
  }

  bool PluginChanged(std::filesystem::file_time_type& outWriteTime) const {
    if (cfg_.seasonModulePluginPath.empty()) {
      return false;
    }
    const bool exists = QueryFileWriteTime(cfg_.seasonModulePluginPath, outWriteTime);

    if (!initialized_) {
      return true;
    }
    if (exists != pluginWriteTimeKnown_) {
      return true;
    }
    if (exists && outWriteTime != pluginWriteTime_) {
      return true;
    }
    return false;
  }

  void RebuildAdapter() {
    adapter_.reset();
    module_.reset();

    if (!cfg_.seasonModulePluginPath.empty()) {
      module_ = CreateSeasonModuleFromPlugin(cfg_.seasonModulePluginPath);
    }
    if (module_ == nullptr) {
      module_ = CreateSeasonModule(cfg_.sceneProfile);
    }
    if (module_ == nullptr) {
      module_ = CreateSeasonModule("default");
    }

    if (module_ != nullptr) {
      adapter_ = module_->CreateWorldAdapter(cfg_);
    }
    if (adapter_ == nullptr) {
      adapter_ = CreateRenderWorldAdapterFromSceneBuilder(CreateDefaultSceneModelBuilder(cfg_));
    }

    if (auto descriptor = LoadSceneDescriptorForProfile(cfg_); descriptor.has_value()) {
      adapter_ = std::make_unique<DescriptorDecoratingRenderWorldAdapter>(std::move(adapter_), std::move(*descriptor));
    }

    descriptorPath_ = ResolveSceneDescriptorPathForProfile(cfg_);
    descriptorWriteTimeKnown_ = QueryFileWriteTime(descriptorPath_, descriptorWriteTime_);

    pluginWriteTimeKnown_ = QueryFileWriteTime(cfg_.seasonModulePluginPath, pluginWriteTime_);
    initialized_ = true;
  }

  void MaybeRebuildIfNeeded() {
    std::string descriptorPath;
    std::filesystem::file_time_type descriptorWriteTime{};
    std::filesystem::file_time_type pluginWriteTime{};

    bool shouldRebuild = !initialized_;
    if (cfg_.hotReloadSceneDescriptor || !initialized_) {
      shouldRebuild = shouldRebuild || DescriptorChanged(descriptorPath, descriptorWriteTime);
    }
    if (cfg_.hotReloadSeasonModule || !initialized_) {
      shouldRebuild = shouldRebuild || PluginChanged(pluginWriteTime);
    }

    if (!shouldRebuild) {
      return;
    }

    (void)descriptorPath;
    (void)descriptorWriteTime;
    RebuildAdapter();
  }

  ViewerConfig cfg_;
  std::unique_ptr<ISeasonModule> module_;
  std::unique_ptr<IRenderWorldAdapter> adapter_;
  bool initialized_ = false;

  std::string descriptorPath_;
  std::filesystem::file_time_type descriptorWriteTime_{};
  bool descriptorWriteTimeKnown_ = false;

  std::filesystem::file_time_type pluginWriteTime_{};
  bool pluginWriteTimeKnown_ = false;
};

}  // namespace

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
  return std::make_unique<HotReloadingRenderWorldAdapter>(cfg);
}

}  // namespace repulsor3d
