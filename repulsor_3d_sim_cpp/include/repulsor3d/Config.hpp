#pragma once

#include <string>

namespace repulsor3d {

struct ViewerConfig {
  std::string sceneProfile = "2026Rebuilt";
  std::string renderPipelinePath = "";
  std::string sceneDescriptorPath = "";
  std::string seasonModulePluginPath = "";
  std::string renderFeaturePluginPath = "";
  bool hotReloadSceneDescriptor = true;
  bool hotReloadSeasonModule = true;
  std::string runtimeConfigProfilePath = "";
  bool hotReloadRuntimeConfigProfile = true;

  std::string ntServer = "127.0.0.1";
  std::string ntClientName = "repulsor_3d_sim_cpp";
  std::string dataSourceType = "auto";
  std::string dataSourcePluginPath = "";
  std::string replaySnapshotPath = "";
  bool replayLoop = true;
  std::string recordSnapshotPath = "";
  std::string fieldVisionPath = "FieldVision/main";
  std::string repulsorVisionPath = "RepulsorVision";
  std::string poseBasePath = "AdvantageKit/RealOutputs/Odometry";
  std::string poseStructKey = "Robot";
  std::string ntSchemaPath = "";
  bool hotReloadNtSchema = true;

  std::string truthSocketHost = "127.0.0.1";
  int truthSocketPort = 5809;
  bool truthSocketEnabled = true;

  std::string fieldImagePath = "field.png";
  bool showFieldImage = true;
  float fieldImageAlpha = 0.92F;
  bool fieldImageFlipX = true;
  bool fieldImageFlipY = false;

  float collectAgeDecay = 1.25F;
  float resourceHardMaxAgeS = 0.95F;
  bool showAgeFilteredFuel = false;

  int windowW = 1280;
  int windowH = 720;
  int fps = 60;

  float fieldLengthM = 16.540988F;
  float fieldWidthM = 8.21055F;
  float fieldZM = 0.0F;

  float ballRadiusM = 0.075F;
  float obsBoxSideM = 0.6F;

  float robotBoxLM = 0.85F;
  float robotBoxWM = 0.85F;
  float robotBoxHM = 0.35F;

  bool showRobotCadModel = false;
  std::string robotCadModelPath = "";
  float robotCadScaleM = 1.0F;
  float robotCadZOffsetM = 0.0F;

  bool showFieldCadModel = true;
  std::string fieldCadModelPath = "field_2026rebuilt.gltf";
  float fieldCadScaleM = 1.0F;
  bool fieldCadFlipX = true;
  float fieldCadZOffsetM = 0.0F;
  float fieldCadOffsetXM = 0.0F;
  float fieldCadOffsetYM = 0.0F;

  // Incoming world-coordinate frame mapping into render-space meters.
  // Preset values:
  // - "top_right_negative": incoming (0,0)=top-right; (-fieldLength,-fieldWidth)=bottom-left
  // - "custom": use the affine parameters below
  std::string incomingCoordFrame = "top_right_negative";
  std::string incomingCoordCalibrationProfilePath = "";
  std::string incomingCoordCalibrationProfileName = "default";
  float incomingCoordOriginXM = 0.0F;
  float incomingCoordOriginYM = 0.0F;
  float incomingCoordRotationDeg = 0.0F;
  float incomingCoordScaleMPerUnit = 1.0F;
  float incomingCoordZScaleMPerUnit = 1.0F;

  float cameraDistanceM = 12.0F;
  float cameraPitchDeg = 35.0F;
  float cameraYawDeg = 135.0F;

  bool followRobot = true;
  bool showCameraDebug = false;
  bool showTruthFuel = false;
  bool showDiagnostics = true;
  bool showDebugPanel = true;
  int maxRenderFuel = 250;
  int maxRenderTruthFuel = 150;
  int maxCameraDebugRaysPerCamera = 200;
  bool vsync = false;

  float followSmoothTimeS = 0.18F;
  float followMaxSpeedMps = 12.0F;

  float robotSmoothTimeS = 0.10F;
  float robotMaxSpeedMps = 20.0F;
};

ViewerConfig LoadConfigFromEnv();

}  // namespace repulsor3d
