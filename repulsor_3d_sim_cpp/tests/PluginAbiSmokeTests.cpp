#include <iostream>
#include <cstdlib>
#include <string>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/modules/SeasonModule.hpp"
#include "repulsor3d/sim/SimWorld.hpp"

#ifndef REPULSOR_TEST_PLUGIN_PATH
#define REPULSOR_TEST_PLUGIN_PATH ""
#endif

int main() {
  std::string pluginPath = REPULSOR_TEST_PLUGIN_PATH;
  if (pluginPath.empty()) {
    const char* envPath = std::getenv("REPULSOR_TEST_PLUGIN_PATH");
    if (envPath != nullptr) {
      pluginPath = envPath;
    }
  }
  if (pluginPath.empty()) {
    std::cerr << "REPULSOR_TEST_PLUGIN_PATH is not set\n";
    return 10;
  }

  auto module = repulsor3d::CreateSeasonModuleFromPlugin(pluginPath);
  if (module == nullptr) {
    std::cerr << "Failed to load season plugin from: " << pluginPath << "\n";
    return 11;
  }

  if (module->Id().empty()) {
    std::cerr << "Loaded plugin returned empty module id\n";
    return 12;
  }

  repulsor3d::ViewerConfig cfg;
  cfg.sceneProfile = "2026Rebuilt";

  auto adapter = module->CreateWorldAdapter(cfg);
  if (adapter == nullptr) {
    std::cerr << "Plugin module did not create a world adapter\n";
    return 13;
  }

  repulsor3d::SnapshotBundle bundle;
  bundle.pieces = 4;
  bundle.method = "plugin-abi-test";
  repulsor3d::SnapshotBundleSimWorld world(bundle);

  repulsor3d::SceneToggleState toggles;
  toggles.showFieldImage = true;
  const auto frame = adapter->BuildFrame(world, toggles);
  if (frame.overlayLines.empty()) {
    std::cerr << "Plugin world adapter produced no overlay output\n";
    return 14;
  }

  std::cout << "Plugin ABI smoke tests passed\n";
  return 0;
}
