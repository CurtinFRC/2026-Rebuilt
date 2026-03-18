#pragma once

#include <memory>
#include <string>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/render/RenderWorldAdapter.hpp"
#include "repulsor3d/render/SceneModelBuilder.hpp"

namespace repulsor3d {

class ISeasonDefinition {
 public:
  virtual ~ISeasonDefinition() = default;

  virtual std::string Id() const = 0;
  virtual std::unique_ptr<ISceneModelBuilder> CreateSceneModelBuilder(const ViewerConfig& cfg) const = 0;
  virtual std::string DefaultNtSchemaPath() const { return {}; }
  virtual std::string DefaultCoordinateProfilePath() const { return {}; }
  virtual void ApplyDefaults(ViewerConfig& /*cfg*/) const {}

  virtual std::unique_ptr<IRenderWorldAdapter> CreateWorldAdapter(const ViewerConfig& cfg) const {
    auto builder = CreateSceneModelBuilder(cfg);
    if (builder == nullptr) {
      return nullptr;
    }
    return CreateRenderWorldAdapterFromSceneBuilder(std::move(builder));
  }
};

}  // namespace repulsor3d
