#pragma once

#include <optional>
#include <string>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/domain/CoordinateFrameMapper.hpp"

namespace repulsor3d {

struct CoordinateCalibrationProfile {
  std::string name = "default";
  CoordinateFrameMapper::Config mapper;
};

std::optional<CoordinateCalibrationProfile> LoadCoordinateCalibrationProfile(
    const std::string& filePath,
    const std::string& profileName);

CoordinateFrameMapper::Config ResolveCoordinateMapperConfigFromProfile(
    const ViewerConfig& cfg,
    const CoordinateFrameMapper::Config& fallback);

}  // namespace repulsor3d

