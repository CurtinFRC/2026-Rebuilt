#pragma once

#include <unordered_map>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/render/SceneModelBuilder.hpp"

namespace repulsor3d {

class Season2026RebuiltModelBuilder final : public ISceneModelBuilder {
 public:
  explicit Season2026RebuiltModelBuilder(const ViewerConfig& cfg);

  RenderSceneFrame BuildFrame(const SnapshotBundle& bundle, const SceneToggleState& toggles) override;

 private:
  void AppendFuelPrimitives(RenderSceneFrame& frame, const WorldSnapshot& snap, bool showAgeFilteredFuel);
  void AppendTruthFuelPrimitives(RenderSceneFrame& frame, const WorldSnapshot& snap);
  void AppendObstaclePrimitives(RenderSceneFrame& frame, const WorldSnapshot& snap) const;
  void AppendRobotPrimitives(RenderSceneFrame& frame, const WorldSnapshot& snap) const;
  void AppendCameraPrimitives(RenderSceneFrame& frame, const WorldSnapshot& snap) const;
  void AppendCadModelPrimitives(RenderSceneFrame& frame, const WorldSnapshot& snap) const;

  static std::string NormalizeType(const std::string& type);

  ViewerConfig cfg_;

  float fieldZ_ = 0.0F;
  float fuelRadius_ = 0.0F;
  float obsSide_ = 0.0F;
  float robotL_ = 0.0F;
  float robotW_ = 0.0F;
  float robotH_ = 0.0F;

  bool showRobotCadModel_ = false;
  std::string robotCadModelPath_;
  float robotCadScaleM_ = 1.0F;
  float robotCadZOffsetM_ = 0.0F;

  bool showFieldCadModel_ = false;
  std::string fieldCadModelPath_;
  float fieldCadScaleM_ = 1.0F;
  bool fieldCadFlipX_ = true;
  float fieldCadZOffsetM_ = 0.0F;
  float fieldCadOffsetXM_ = 0.0F;
  float fieldCadOffsetYM_ = 0.0F;

  glm::vec4 colFuel_{1.0F, 0.95F, 0.15F, 0.95F};
  glm::vec4 colTruthFuel_{0.15F, 0.95F, 0.35F, 0.70F};
  glm::vec4 colOther_{1.0F, 0.15F, 0.15F, 0.85F};
  glm::vec4 colUs_{1.0F, 0.27F, 0.0F, 0.75F};
  glm::vec4 colActive_{0.15F, 0.9F, 0.3F, 0.9F};
  glm::vec4 colChosen_{0.15F, 0.3F, 0.9F, 0.9F};
  glm::vec4 colFinal_{0.9F, 0.3F, 0.15F, 0.9F};
  glm::vec4 colHeading_{1.0F, 0.1F, 0.1F, 0.9F};
  glm::vec4 colCam_{0.20F, 0.75F, 1.0F, 0.85F};
  glm::vec4 colCamFov_{0.20F, 0.75F, 1.0F, 0.55F};
  glm::vec4 colCamRayOk_{0.20F, 0.95F, 0.35F, 0.55F};
  glm::vec4 colCamRayBad_{1.00F, 0.25F, 0.25F, 0.40F};

  std::unordered_map<std::string, double> fuelLastSeen_;
  std::unordered_map<std::string, FieldVisionObject> fuelCache_;
};

}  // namespace repulsor3d
