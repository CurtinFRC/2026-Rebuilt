#pragma once

#include <functional>
#include <memory>
#include <string>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/season/SeasonDefinition.hpp"

namespace repulsor3d {

using SeasonDefinitionFactoryFn = std::function<std::unique_ptr<ISeasonDefinition>()>;

void RegisterSeasonDefinition(const std::string& seasonId, SeasonDefinitionFactoryFn factoryFn);
std::unique_ptr<ISeasonDefinition> CreateSeasonDefinition(const std::string& seasonId);
std::unique_ptr<ISeasonDefinition> CreateDefaultSeasonDefinition(const ViewerConfig& cfg);

}  // namespace repulsor3d
