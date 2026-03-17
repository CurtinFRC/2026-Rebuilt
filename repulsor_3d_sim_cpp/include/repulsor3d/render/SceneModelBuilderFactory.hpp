#pragma once

#include <functional>
#include <memory>
#include <string>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/render/SceneModelBuilder.hpp"

namespace repulsor3d {

using SceneModelBuilderFactoryFn = std::function<std::unique_ptr<ISceneModelBuilder>(const ViewerConfig&)>;

void RegisterSceneModelBuilder(const std::string& sceneProfile, SceneModelBuilderFactoryFn fn);
std::unique_ptr<ISceneModelBuilder> CreateSceneModelBuilder(const std::string& sceneProfile, const ViewerConfig& cfg);
std::unique_ptr<ISceneModelBuilder> CreateDefaultSceneModelBuilder(const ViewerConfig& cfg);

}  // namespace repulsor3d
