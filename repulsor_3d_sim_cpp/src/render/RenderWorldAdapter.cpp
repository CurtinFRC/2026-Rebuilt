#include "repulsor3d/render/RenderWorldAdapter.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <utility>

#include "repulsor3d/app/composition/SeasonWorldComposition.hpp"
#include "repulsor3d/modules/SeasonModule.hpp"
#include "repulsor3d/render/SceneDescriptor.hpp"
#include "repulsor3d/render/templates/GenericSeasonModelBuilderTemplate.hpp"

namespace repulsor3d {
namespace {

std::string LowerAscii(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

double DynamicDoubleOr(
    const DynamicEntityRecord& record,
    const std::string& key,
    const double fallback) {
  if (key.empty()) {
    return fallback;
  }
  if (const auto it = record.doubles.find(key); it != record.doubles.end()) {
    return it->second;
  }
  return fallback;
}

std::string DynamicStringOr(
    const DynamicEntityRecord& record,
    const std::string& key,
    const std::string& fallback) {
  if (key.empty()) {
    return fallback;
  }
  if (const auto it = record.strings.find(key); it != record.strings.end()) {
    return it->second;
  }
  return fallback;
}

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
    auto composed = app::composition::ComposeSeasonWorldAdapter(cfg_);
    module_ = std::move(composed.module);
    adapter_ = std::move(composed.adapter);

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

  if (!descriptor_.dynamicEntityBindings.empty()) {
    const SnapshotBundle snapshotBundle = world.AsSnapshotBundle();
    for (const auto& binding : descriptor_.dynamicEntityBindings) {
      const auto channelIt = snapshotBundle.snapshot.dynamicEntityGroups.find(binding.channel);
      if (channelIt == snapshotBundle.snapshot.dynamicEntityGroups.end()) {
        continue;
      }

      const std::string entityType = LowerAscii(binding.entityType);
      for (std::size_t i = 0; i < channelIt->second.size(); ++i) {
        const auto& record = channelIt->second[i];
        const std::string id = binding.idPrefix + (record.id.empty() ? std::to_string(i) : record.id);

        const glm::vec3 position{
            static_cast<float>(DynamicDoubleOr(record, binding.xKey, 0.0)),
            static_cast<float>(DynamicDoubleOr(record, binding.yKey, 0.0)),
            static_cast<float>(DynamicDoubleOr(record, binding.zKey, 0.0)),
        };
        const float yawDeg = static_cast<float>(DynamicDoubleOr(record, binding.yawDegKey, 0.0));

        if (entityType == "sphere") {
          SpherePrimitive sphere;
          sphere.center = position;
          sphere.radius = static_cast<float>(DynamicDoubleOr(record, "radius", binding.defaultRadius));
          sphere.color = binding.color;
          sphere.pass = binding.pass;
          frame.entities.push_back(
              {.id = id,
               .pass = binding.pass,
               .payload = sphere,
               .parentId = "",
               .transform = {},
               .hasTransform = false,
               .culling = binding.culling});
        } else if (entityType == "box") {
          BoxPrimitive box;
          box.center = position;
          box.size = glm::vec3{
              static_cast<float>(DynamicDoubleOr(record, "size_x", binding.defaultSize.x)),
              static_cast<float>(DynamicDoubleOr(record, "size_y", binding.defaultSize.y)),
              static_cast<float>(DynamicDoubleOr(record, "size_z", binding.defaultSize.z)),
          };
          box.yawDeg = yawDeg;
          box.color = binding.color;
          box.pass = binding.pass;
          frame.entities.push_back(
              {.id = id,
               .pass = binding.pass,
               .payload = box,
               .parentId = "",
               .transform = {},
               .hasTransform = false,
               .culling = binding.culling});
        } else if (entityType == "mesh" && !binding.assetPath.empty()) {
          MeshInstancePrimitive mesh;
          mesh.assetPath = binding.assetPath;
          mesh.position = position;
          mesh.rotationDeg = glm::vec3{0.0F, 0.0F, yawDeg};
          mesh.scale = glm::vec3{
              static_cast<float>(DynamicDoubleOr(record, "scale_x", binding.defaultScale.x)),
              static_cast<float>(DynamicDoubleOr(record, "scale_y", binding.defaultScale.y)),
              static_cast<float>(DynamicDoubleOr(record, "scale_z", binding.defaultScale.z)),
          };
          mesh.color = binding.color;
          mesh.useAssetColor = binding.useAssetColor;
          mesh.wireframe = binding.wireframe;
          mesh.roughnessOverride = static_cast<float>(DynamicDoubleOr(record, "roughness", binding.roughnessOverride));
          mesh.metallicOverride = static_cast<float>(DynamicDoubleOr(record, "metallic", binding.metallicOverride));
          mesh.normalStrength = static_cast<float>(DynamicDoubleOr(record, "normal_strength", binding.normalStrength));
          mesh.albedoTexturePath = DynamicStringOr(record, "albedo_texture_path", binding.albedoTexturePath);
          mesh.normalTexturePath = DynamicStringOr(record, "normal_texture_path", binding.normalTexturePath);
          mesh.pass = binding.pass;
          frame.entities.push_back(
              {.id = id,
               .pass = binding.pass,
               .payload = mesh,
               .parentId = "",
               .transform = {},
               .hasTransform = false,
               .culling = binding.culling});
        } else if (entityType == "overlay") {
          OverlayLine overlay;
          overlay.text = DynamicStringOr(record, binding.textKey, id);
          overlay.color = binding.color;
          overlay.anchor = OverlayAnchor::BottomLeft;
          overlay.marginX = 10.0F;
          overlay.marginY = 14.0F;
          frame.entities.push_back(
              {.id = id,
               .pass = RenderPass::Overlay,
               .payload = overlay,
               .parentId = "",
               .transform = {},
               .hasTransform = false,
               .culling = {.enabled = false, .boundsRadius = 0.0F}});
        }
      }
    }
  }
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
