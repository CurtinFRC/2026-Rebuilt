#include "repulsor3d/domain/CoordinateCalibrationProfile.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>

#include <nlohmann/json.hpp>

namespace repulsor3d {
namespace {

std::string ToLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

CoordinateFrameMapper::Config ParseMapperConfig(const nlohmann::json& node, const CoordinateFrameMapper::Config& fallback) {
  CoordinateFrameMapper::Config out = fallback;
  if (!node.is_object()) {
    return out;
  }
  out.originXM = node.value("originXM", out.originXM);
  out.originYM = node.value("originYM", out.originYM);
  out.rotationDeg = node.value("rotationDeg", out.rotationDeg);
  out.scaleMPerUnit = node.value("scaleMPerUnit", out.scaleMPerUnit);
  out.zScaleMPerUnit = node.value("zScaleMPerUnit", out.zScaleMPerUnit);
  return out;
}

}  // namespace

std::optional<CoordinateCalibrationProfile> LoadCoordinateCalibrationProfile(
    const std::string& filePath,
    const std::string& profileName) {
  if (filePath.empty()) {
    return std::nullopt;
  }

  std::ifstream in(filePath);
  if (!in.is_open()) {
    return std::nullopt;
  }

  nlohmann::json root;
  try {
    in >> root;
  } catch (...) {
    return std::nullopt;
  }
  if (!root.is_object()) {
    return std::nullopt;
  }

  const std::string wanted = ToLower(profileName.empty() ? "default" : profileName);
  CoordinateFrameMapper::Config defaults;
  if (root.contains("defaults")) {
    defaults = ParseMapperConfig(root["defaults"], defaults);
  }

  if (root.contains("profiles") && root["profiles"].is_object()) {
    for (auto it = root["profiles"].begin(); it != root["profiles"].end(); ++it) {
      if (ToLower(it.key()) != wanted) {
        continue;
      }
      CoordinateCalibrationProfile profile;
      profile.name = it.key();
      profile.mapper = ParseMapperConfig(it.value(), defaults);
      return profile;
    }
  }

  if (root.contains("profile") && root["profile"].is_object()) {
    CoordinateCalibrationProfile profile;
    profile.name = profileName.empty() ? "default" : profileName;
    profile.mapper = ParseMapperConfig(root["profile"], defaults);
    return profile;
  }

  return std::nullopt;
}

CoordinateFrameMapper::Config ResolveCoordinateMapperConfigFromProfile(
    const ViewerConfig& cfg,
    const CoordinateFrameMapper::Config& fallback) {
  if (cfg.incomingCoordCalibrationProfilePath.empty()) {
    return fallback;
  }
  auto profile = LoadCoordinateCalibrationProfile(
      cfg.incomingCoordCalibrationProfilePath,
      cfg.incomingCoordCalibrationProfileName);
  if (!profile.has_value()) {
    return fallback;
  }
  return profile->mapper;
}

}  // namespace repulsor3d

