#pragma once

#include <string>

#include "repulsor3d/Config.hpp"

namespace repulsor3d {

bool LoadViewerConfigProfile(const std::string& filePath, ViewerConfig& inOutConfig, std::string* outError);

}  // namespace repulsor3d
