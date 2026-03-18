#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/Model.hpp"
#include "repulsor3d/render/Season2026RebuiltModelBuilder.hpp"

namespace {

std::string ReadTextFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    return {};
  }
  std::ostringstream out;
  out << in.rdbuf();
  return out.str();
}

std::string CanonicalFrameSignature(const repulsor3d::RenderSceneFrame& frame) {
  std::vector<std::string> lines;
  lines.reserve(frame.entities.size() + frame.overlayLines.size() + 8);
  lines.push_back(std::string("drawFieldImage=") + (frame.drawFieldImage ? "1" : "0"));
  lines.push_back(std::string("drawGrid=") + (frame.drawGrid ? "1" : "0"));
  lines.push_back(std::string("drawAxes=") + (frame.drawAxes ? "1" : "0"));
  lines.push_back("overlays=" + std::to_string(frame.overlayLines.size()));
  lines.push_back("entities=" + std::to_string(frame.entities.size()));

  for (const auto& entity : frame.entities) {
    std::ostringstream line;
    line << "entity id=" << entity.id << " pass=" << static_cast<int>(entity.pass);
    std::visit(
        [&](const auto& payload) {
          using T = std::decay_t<decltype(payload)>;
          if constexpr (std::is_same_v<T, repulsor3d::SpherePrimitive>) {
            line << " type=sphere r=" << payload.radius;
          } else if constexpr (std::is_same_v<T, repulsor3d::BoxPrimitive>) {
            line << " type=box sx=" << payload.size.x << " sy=" << payload.size.y << " sz=" << payload.size.z;
          } else if constexpr (std::is_same_v<T, repulsor3d::LinePrimitive>) {
            line << " type=line w=" << payload.width;
          } else if constexpr (std::is_same_v<T, repulsor3d::MeshInstancePrimitive>) {
            line << " type=mesh asset=" << payload.assetPath;
          } else if constexpr (std::is_same_v<T, repulsor3d::OverlayLine>) {
            line << " type=overlay text=" << payload.text;
          }
        },
        entity.payload);
    lines.push_back(line.str());
  }
  std::sort(lines.begin(), lines.end());

  std::ostringstream out;
  for (const auto& line : lines) {
    out << line << "\n";
  }
  return out.str();
}

int Run2026RebuiltGoldenFrameRegression() {
  repulsor3d::ViewerConfig cfg;
  cfg.sceneProfile = "2026Rebuilt";
  cfg.showFieldCadModel = true;
  cfg.fieldCadModelPath = "field_2026rebuilt.gltf";
  cfg.showRobotCadModel = true;
  cfg.robotCadModelPath = "field_2026rebuilt.gltf";

  repulsor3d::SnapshotBundle bundle;
  bundle.connected = true;
  bundle.pieces = 3;
  bundle.method = "regression_test";
  bundle.snapshot.pose = repulsor3d::Pose2D{1.25, -0.75, 0.35};
  bundle.snapshot.activeGoal = repulsor3d::Pose2D{2.1, 1.2, -0.4};
  bundle.snapshot.finalCollect = repulsor3d::Pose2D{-1.3, 0.8, 0.2};

  repulsor3d::FieldVisionObject fuelA;
  fuelA.oid = "fuel_a";
  fuelA.type = "fuel";
  fuelA.x = 0.4;
  fuelA.y = -0.6;
  fuelA.z = 0.05;
  bundle.snapshot.fieldVision.push_back(fuelA);

  repulsor3d::RepulsorVisionObstacle obs;
  obs.oid = "obs_0";
  obs.kind = "generic";
  obs.x = 0.8;
  obs.y = -0.4;
  obs.sizeX = 0.9;
  obs.sizeY = 0.7;
  bundle.snapshot.repulsorVision.push_back(obs);

  repulsor3d::SceneToggleState toggles;
  toggles.showFieldImage = false;
  toggles.showTruthFuel = true;
  toggles.showAgeFilteredFuel = true;
  toggles.showCameraDebug = true;

  repulsor3d::Season2026RebuiltModelBuilder builder(cfg);
  const repulsor3d::RenderSceneFrame frame = builder.BuildFrame(bundle, toggles);
  const std::string signature = CanonicalFrameSignature(frame);

  std::string goldenPath = "tests/golden/2026rebuilt_frame_signature.txt";
#ifdef REPULSOR_TEST_SOURCE_DIR
  goldenPath = std::string(REPULSOR_TEST_SOURCE_DIR) + "/tests/golden/2026rebuilt_frame_signature.txt";
#endif
  const std::string golden = ReadTextFile(goldenPath);
  if (golden.empty()) {
    std::cerr << "Golden file missing or empty: " << goldenPath << "\n";
    std::cerr << "Generated signature:\n" << signature << "\n";
    return 2;
  }
  if (signature != golden) {
    std::cerr << "Frame signature regression mismatch against " << goldenPath << "\n";
    std::cerr << "Expected:\n" << golden << "\n";
    std::cerr << "Actual:\n" << signature << "\n";
    return 3;
  }
  return 0;
}

}  // namespace

int main() {
  if (const int rc = Run2026RebuiltGoldenFrameRegression(); rc != 0) {
    return rc;
  }
  std::cout << "Render regression tests passed\n";
  return 0;
}
