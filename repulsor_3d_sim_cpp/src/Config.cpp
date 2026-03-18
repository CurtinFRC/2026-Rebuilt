#include "repulsor3d/Config.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <string>

namespace repulsor3d {
namespace {

std::string GetEnvString(const char* name, const std::string& fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  return value;
}

int GetEnvInt(const char* name, const int fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  try {
    return std::stoi(value);
  } catch (...) {
    return fallback;
  }
}

float GetEnvFloat(const char* name, const float fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  try {
    return std::stof(value);
  } catch (...) {
    return fallback;
  }
}

bool GetEnvBool(const char* name, const bool fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }

  std::string v(value);
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (v == "1" || v == "true" || v == "yes" || v == "y" || v == "on") {
    return true;
  }
  if (v == "0" || v == "false" || v == "no" || v == "n" || v == "off") {
    return false;
  }
  return fallback;
}

}  // namespace

ViewerConfig LoadConfigFromEnv() {
  ViewerConfig cfg;

  cfg.sceneProfile = GetEnvString("SIM_SCENE_PROFILE", cfg.sceneProfile);
  cfg.sceneDescriptorPath = GetEnvString("SCENE_DESCRIPTOR_PATH", cfg.sceneDescriptorPath);
  cfg.seasonModulePluginPath = GetEnvString("SEASON_MODULE_PLUGIN_PATH", cfg.seasonModulePluginPath);
  cfg.hotReloadSceneDescriptor = GetEnvBool("HOT_RELOAD_SCENE_DESCRIPTOR", cfg.hotReloadSceneDescriptor);
  cfg.hotReloadSeasonModule = GetEnvBool("HOT_RELOAD_SEASON_MODULE", cfg.hotReloadSeasonModule);

  cfg.ntServer = GetEnvString("NT_SERVER", cfg.ntServer);
  cfg.ntClientName = GetEnvString("NT_CLIENT_NAME", cfg.ntClientName);
  cfg.fieldVisionPath = GetEnvString("NT_FIELDVISION_PATH", cfg.fieldVisionPath);
  cfg.repulsorVisionPath = GetEnvString("NT_REPULSORVISION_PATH", cfg.repulsorVisionPath);
  cfg.poseBasePath = GetEnvString("NT_POSE_BASE_PATH", cfg.poseBasePath);
  cfg.poseStructKey = GetEnvString("NT_POSE_STRUCT_KEY", cfg.poseStructKey);

  cfg.truthSocketHost = GetEnvString("TRUTH_SOCKET_HOST", cfg.truthSocketHost);
  cfg.truthSocketPort = GetEnvInt("TRUTH_SOCKET_PORT", cfg.truthSocketPort);
  cfg.truthSocketEnabled = GetEnvBool("TRUTH_SOCKET_ENABLED", cfg.truthSocketEnabled);

  cfg.fieldImagePath = GetEnvString("FIELD_IMAGE_PATH", cfg.fieldImagePath);
  cfg.showFieldImage = GetEnvBool("SHOW_FIELD_IMAGE", cfg.showFieldImage);
  cfg.fieldImageAlpha = GetEnvFloat("FIELD_IMAGE_ALPHA", cfg.fieldImageAlpha);
  cfg.fieldImageFlipX = GetEnvBool("FIELD_IMAGE_FLIP_X", cfg.fieldImageFlipX);
  cfg.fieldImageFlipY = GetEnvBool("FIELD_IMAGE_FLIP_Y", cfg.fieldImageFlipY);

  cfg.collectAgeDecay = GetEnvFloat("COLLECT_AGE_DECAY", cfg.collectAgeDecay);
  cfg.resourceHardMaxAgeS = GetEnvFloat("RESOURCE_HARD_MAX_AGE_S", cfg.resourceHardMaxAgeS);
  cfg.showAgeFilteredFuel = GetEnvBool("SHOW_AGE_FILTERED_FUEL", cfg.showAgeFilteredFuel);

  cfg.windowW = GetEnvInt("WINDOW_W", cfg.windowW);
  cfg.windowH = GetEnvInt("WINDOW_H", cfg.windowH);
  cfg.fps = GetEnvInt("FPS", cfg.fps);

  cfg.fieldLengthM = GetEnvFloat("FIELD_LENGTH_M", cfg.fieldLengthM);
  cfg.fieldWidthM = GetEnvFloat("FIELD_WIDTH_M", cfg.fieldWidthM);
  cfg.fieldZM = GetEnvFloat("FIELD_Z_M", cfg.fieldZM);

  cfg.ballRadiusM = GetEnvFloat("BALL_RADIUS_M", cfg.ballRadiusM);
  cfg.obsBoxSideM = GetEnvFloat("OBS_BOX_SIDE_M", cfg.obsBoxSideM);

  cfg.robotBoxLM = GetEnvFloat("ROBOT_BOX_L_M", cfg.robotBoxLM);
  cfg.robotBoxWM = GetEnvFloat("ROBOT_BOX_W_M", cfg.robotBoxWM);
  cfg.robotBoxHM = GetEnvFloat("ROBOT_BOX_H_M", cfg.robotBoxHM);

  cfg.showRobotCadModel = GetEnvBool("SHOW_ROBOT_CAD_MODEL", cfg.showRobotCadModel);
  cfg.robotCadModelPath = GetEnvString("ROBOT_CAD_MODEL_PATH", cfg.robotCadModelPath);
  cfg.robotCadScaleM = GetEnvFloat("ROBOT_CAD_SCALE_M", cfg.robotCadScaleM);
  cfg.robotCadZOffsetM = GetEnvFloat("ROBOT_CAD_Z_OFFSET_M", cfg.robotCadZOffsetM);

  cfg.showFieldCadModel = GetEnvBool("SHOW_FIELD_CAD_MODEL", cfg.showFieldCadModel);
  cfg.fieldCadModelPath = GetEnvString("FIELD_CAD_MODEL_PATH", cfg.fieldCadModelPath);
  cfg.fieldCadScaleM = GetEnvFloat("FIELD_CAD_SCALE_M", cfg.fieldCadScaleM);
  cfg.fieldCadFlipX = GetEnvBool("FIELD_CAD_FLIP_X", cfg.fieldCadFlipX);
  cfg.fieldCadZOffsetM = GetEnvFloat("FIELD_CAD_Z_OFFSET_M", cfg.fieldCadZOffsetM);
  cfg.fieldCadOffsetXM = GetEnvFloat("FIELD_CAD_OFFSET_X_M", cfg.fieldCadOffsetXM);
  cfg.fieldCadOffsetYM = GetEnvFloat("FIELD_CAD_OFFSET_Y_M", cfg.fieldCadOffsetYM);

  cfg.incomingCoordOriginXM = GetEnvFloat("INCOMING_COORD_ORIGIN_X_M", cfg.incomingCoordOriginXM);
  cfg.incomingCoordOriginYM = GetEnvFloat("INCOMING_COORD_ORIGIN_Y_M", cfg.incomingCoordOriginYM);
  cfg.incomingCoordRotationDeg = GetEnvFloat("INCOMING_COORD_ROTATION_DEG", cfg.incomingCoordRotationDeg);
  cfg.incomingCoordScaleMPerUnit = GetEnvFloat("INCOMING_COORD_SCALE_M_PER_UNIT", cfg.incomingCoordScaleMPerUnit);
  cfg.incomingCoordZScaleMPerUnit = GetEnvFloat("INCOMING_COORD_Z_SCALE_M_PER_UNIT", cfg.incomingCoordZScaleMPerUnit);

  cfg.cameraDistanceM = GetEnvFloat("CAMERA_DISTANCE_M", cfg.cameraDistanceM);
  cfg.cameraPitchDeg = GetEnvFloat("CAMERA_PITCH_DEG", cfg.cameraPitchDeg);
  cfg.cameraYawDeg = GetEnvFloat("CAMERA_YAW_DEG", cfg.cameraYawDeg);

  cfg.followRobot = GetEnvBool("FOLLOW_ROBOT", cfg.followRobot);
  cfg.showCameraDebug = GetEnvBool("SHOW_CAMERA_DEBUG", cfg.showCameraDebug);
  cfg.showTruthFuel = GetEnvBool("SHOW_TRUTH_FUEL", cfg.showTruthFuel);
  cfg.showDiagnostics = GetEnvBool("SHOW_DIAGNOSTICS", cfg.showDiagnostics);

  cfg.followSmoothTimeS = GetEnvFloat("FOLLOW_SMOOTH_TIME_S", cfg.followSmoothTimeS);
  cfg.followMaxSpeedMps = GetEnvFloat("FOLLOW_MAX_SPEED_MPS", cfg.followMaxSpeedMps);
  cfg.robotSmoothTimeS = GetEnvFloat("ROBOT_SMOOTH_TIME_S", cfg.robotSmoothTimeS);
  cfg.robotMaxSpeedMps = GetEnvFloat("ROBOT_MAX_SPEED_MPS", cfg.robotMaxSpeedMps);

  return cfg;
}

}  // namespace repulsor3d
