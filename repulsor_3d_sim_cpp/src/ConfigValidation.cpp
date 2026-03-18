#include "repulsor3d/ConfigValidation.hpp"

namespace repulsor3d {

ConfigValidationResult ValidateConfig(const ViewerConfig& cfg) {
  ConfigValidationResult result;

  if (cfg.windowW <= 0 || cfg.windowH <= 0) {
    result.errors.push_back("WINDOW_W and WINDOW_H must be > 0");
  }
  if (cfg.fps <= 0) {
    result.errors.push_back("FPS must be > 0");
  }
  if (cfg.fieldLengthM <= 0.0F || cfg.fieldWidthM <= 0.0F) {
    result.errors.push_back("FIELD_LENGTH_M and FIELD_WIDTH_M must be > 0");
  }
  if (cfg.cameraDistanceM <= 0.0F) {
    result.errors.push_back("CAMERA_DISTANCE_M must be > 0");
  }
  if (cfg.followSmoothTimeS <= 0.0F) {
    result.errors.push_back("FOLLOW_SMOOTH_TIME_S must be > 0");
  }

  if (cfg.fieldImageAlpha < 0.0F || cfg.fieldImageAlpha > 1.0F) {
    result.warnings.push_back("FIELD_IMAGE_ALPHA should be in [0, 1]");
  }
  if (cfg.robotCadScaleM <= 0.0F && cfg.showRobotCadModel) {
    result.warnings.push_back("ROBOT_CAD_SCALE_M should be > 0 when SHOW_ROBOT_CAD_MODEL is enabled");
  }
  if (cfg.fieldCadScaleM <= 0.0F && cfg.showFieldCadModel) {
    result.warnings.push_back("FIELD_CAD_SCALE_M should be > 0 when SHOW_FIELD_CAD_MODEL is enabled");
  }
  if (cfg.incomingCoordScaleMPerUnit <= 0.0F) {
    result.errors.push_back("INCOMING_COORD_SCALE_M_PER_UNIT must be > 0");
  }
  if (cfg.incomingCoordZScaleMPerUnit <= 0.0F) {
    result.errors.push_back("INCOMING_COORD_Z_SCALE_M_PER_UNIT must be > 0");
  }

  return result;
}

}  // namespace repulsor3d
