#pragma once

#include <string>
#include <vector>

#include "repulsor3d/Config.hpp"

namespace repulsor3d {

struct ConfigValidationResult {
  std::vector<std::string> warnings;
  std::vector<std::string> errors;

  bool Ok() const { return errors.empty(); }
};

ConfigValidationResult ValidateConfig(const ViewerConfig& cfg);

}  // namespace repulsor3d
