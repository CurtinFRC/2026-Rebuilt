#include <fstream>
#include <iostream>
#include <optional>
#include <cstdio>
#include <string>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/modules/SeasonModule.hpp"
#include "repulsor3d/render/RenderWorldAdapter.hpp"
#include "repulsor3d/render/SceneDescriptor.hpp"
#include "repulsor3d/render/templates/GenericSeasonModelBuilderTemplate.hpp"
#include "repulsor3d/sim/SimWorld.hpp"

namespace {

int RunSceneDescriptorParseTests() {
  const std::string path = "test_scene_descriptor.json";
  {
    std::ofstream out(path);
    out << R"({
      "drawGrid": false,
      "overlay": [{"text":"Hello","anchor":"top_right"}],
      "entities": [{"id":"marker","type":"sphere","pass":"transparent","center":[1,2,3],"radius":0.1}]
    })";
  }

  auto descriptor = repulsor3d::LoadSceneDescriptorFromFile(path);
  std::remove(path.c_str());
  if (!descriptor.has_value()) {
    std::cerr << "SceneDescriptor parse failed\n";
    return 10;
  }
  if (!descriptor->drawGrid.has_value() || descriptor->drawGrid.value() != false) {
    std::cerr << "SceneDescriptor drawGrid override missing\n";
    return 11;
  }
  if (descriptor->staticOverlayLines.empty()) {
    std::cerr << "SceneDescriptor overlay parse missing\n";
    return 12;
  }
  if (descriptor->staticEntities.empty()) {
    std::cerr << "SceneDescriptor entities parse missing\n";
    return 13;
  }
  return 0;
}

int RunRenderWorldAdapterTests() {
  repulsor3d::ViewerConfig cfg;
  repulsor3d::SnapshotBundle bundle;
  bundle.pieces = 3;
  bundle.method = "test";
  repulsor3d::SnapshotBundleSimWorld world(bundle);

  auto adapter = repulsor3d::CreateRenderWorldAdapterFromSceneBuilder(
      std::make_unique<repulsor3d::GenericSeasonModelBuilderTemplate>(cfg));
  if (adapter == nullptr) {
    std::cerr << "Render world adapter creation failed\n";
    return 20;
  }

  repulsor3d::SceneToggleState toggles;
  toggles.showFieldImage = true;
  const auto frame = adapter->BuildFrame(world, toggles);
  if (frame.overlayLines.empty()) {
    std::cerr << "Render world adapter frame overlay missing\n";
    return 21;
  }
  return 0;
}

int RunSeasonModuleTests() {
  repulsor3d::ViewerConfig cfg;
  cfg.sceneProfile = "2026Rebuilt";

  auto module = repulsor3d::CreateDefaultSeasonModule(cfg);
  if (module == nullptr) {
    std::cerr << "Default season module creation failed\n";
    return 30;
  }

  auto adapter = module->CreateWorldAdapter(cfg);
  if (adapter == nullptr) {
    std::cerr << "Season module world adapter creation failed\n";
    return 31;
  }

  repulsor3d::SnapshotBundle bundle;
  bundle.pieces = 7;
  bundle.method = "module-test";
  bundle.snapshot.pose = repulsor3d::Pose2D{.x = 1.0, .y = 2.0, .thetaRad = 0.1};
  repulsor3d::SnapshotBundleSimWorld world(bundle);

  repulsor3d::SceneToggleState toggles;
  toggles.showCameraDebug = true;
  toggles.showTruthFuel = true;
  toggles.showAgeFilteredFuel = false;
  toggles.showFieldImage = true;

  const auto frame = adapter->BuildFrame(world, toggles);
  if (frame.overlayLines.empty()) {
    std::cerr << "Season module frame overlay missing\n";
    return 32;
  }
  if (frame.boxes.empty() && frame.entities.empty()) {
    std::cerr << "Season module frame geometry missing\n";
    return 33;
  }

  return 0;
}

}  // namespace

int main() {
  if (const int rc = RunSceneDescriptorParseTests(); rc != 0) {
    return rc;
  }
  if (const int rc = RunRenderWorldAdapterTests(); rc != 0) {
    return rc;
  }
  if (const int rc = RunSeasonModuleTests(); rc != 0) {
    return rc;
  }

  std::cout << "Module abstraction tests passed\n";
  return 0;
}
