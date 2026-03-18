#include <cstdlib>
#include <iostream>
#include <string>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/render/RenderFeaturePlugin.hpp"

int main() {
#ifdef REPULSOR_TEST_RENDER_PLUGIN_PATH
  std::string pluginPath = REPULSOR_TEST_RENDER_PLUGIN_PATH;
#else
  std::string pluginPath;
#endif
  if (pluginPath.empty()) {
    if (const char* envPath = std::getenv("REPULSOR_TEST_RENDER_PLUGIN_PATH"); envPath != nullptr) {
      pluginPath = envPath;
    }
  }
  if (pluginPath.empty()) {
    std::cerr << "Render plugin path is empty\n";
    return 1;
  }

  auto plugin = repulsor3d::CreateRenderFeaturePluginFromPath(pluginPath);
  if (plugin == nullptr) {
    std::cerr << "Failed to load render feature plugin from: " << pluginPath << "\n";
    return 1;
  }
  if (plugin->Id().empty()) {
    std::cerr << "Loaded render plugin returned empty id\n";
    return 1;
  }

  repulsor3d::ViewerConfig cfg;
  auto features = plugin->CreateFeatures(cfg);
  if (features.empty()) {
    std::cerr << "Render plugin produced no features\n";
    return 1;
  }

  std::cout << "Render feature plugin ABI smoke test passed\n";
  return 0;
}

