#include "repulsor3d/config/RuntimeConfigProfile.hpp"

#include <fstream>

#include <nlohmann/json.hpp>

namespace repulsor3d {

bool LoadViewerConfigProfile(const std::string& filePath, ViewerConfig& inOutConfig, std::string* outError) {
  if (filePath.empty()) {
    if (outError != nullptr) {
      *outError = "empty path";
    }
    return false;
  }

  std::ifstream in(filePath);
  if (!in.is_open()) {
    if (outError != nullptr) {
      *outError = "failed to open file";
    }
    return false;
  }

  nlohmann::json root;
  try {
    in >> root;
  } catch (...) {
    if (outError != nullptr) {
      *outError = "invalid json";
    }
    return false;
  }

  if (!root.is_object()) {
    if (outError != nullptr) {
      *outError = "profile root is not an object";
    }
    return false;
  }

  inOutConfig.sceneProfile = root.value("sceneProfile", inOutConfig.sceneProfile);
  inOutConfig.fieldImagePath = root.value("fieldImagePath", inOutConfig.fieldImagePath);
  inOutConfig.showFieldImage = root.value("showFieldImage", inOutConfig.showFieldImage);
  inOutConfig.showFieldCadModel = root.value("showFieldCadModel", inOutConfig.showFieldCadModel);
  inOutConfig.fieldCadModelPath = root.value("fieldCadModelPath", inOutConfig.fieldCadModelPath);
  inOutConfig.fieldCadScaleM = root.value("fieldCadScaleM", inOutConfig.fieldCadScaleM);
  inOutConfig.fieldCadFlipX = root.value("fieldCadFlipX", inOutConfig.fieldCadFlipX);
  inOutConfig.fieldCadOffsetXM = root.value("fieldCadOffsetXM", inOutConfig.fieldCadOffsetXM);
  inOutConfig.fieldCadOffsetYM = root.value("fieldCadOffsetYM", inOutConfig.fieldCadOffsetYM);
  inOutConfig.fieldLengthM = root.value("fieldLengthM", inOutConfig.fieldLengthM);
  inOutConfig.fieldWidthM = root.value("fieldWidthM", inOutConfig.fieldWidthM);
  inOutConfig.incomingCoordFrame = root.value("incomingCoordFrame", inOutConfig.incomingCoordFrame);
  inOutConfig.incomingCoordOriginXM = root.value("incomingCoordOriginXM", inOutConfig.incomingCoordOriginXM);
  inOutConfig.incomingCoordOriginYM = root.value("incomingCoordOriginYM", inOutConfig.incomingCoordOriginYM);
  inOutConfig.incomingCoordRotationDeg = root.value("incomingCoordRotationDeg", inOutConfig.incomingCoordRotationDeg);
  inOutConfig.incomingCoordScaleMPerUnit = root.value("incomingCoordScaleMPerUnit", inOutConfig.incomingCoordScaleMPerUnit);
  inOutConfig.incomingCoordZScaleMPerUnit =
      root.value("incomingCoordZScaleMPerUnit", inOutConfig.incomingCoordZScaleMPerUnit);
  inOutConfig.runtimeConfigProfilePath = filePath;

  return true;
}

}  // namespace repulsor3d
