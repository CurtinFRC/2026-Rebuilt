#include <cstdlib>
#include <iostream>
#include <string>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/DataSourceFactory.hpp"
#include "repulsor3d/modules/SeasonModule.hpp"
#include "repulsor3d/render/RenderFeaturePlugin.hpp"

namespace {

std::string ReadEnvOrEmpty(const char* key) {
  if (const char* value = std::getenv(key); value != nullptr) {
    return value;
  }
  return {};
}

}  // namespace

int main() {
  const std::string invalidSeasonNoSdk = ReadEnvOrEmpty("REPULSOR_TEST_INVALID_SEASON_NO_SDK_PLUGIN_PATH");
  const std::string invalidDataSourceBadFlavor = ReadEnvOrEmpty("REPULSOR_TEST_INVALID_DATASOURCE_BAD_FLAVOR_PLUGIN_PATH");
  const std::string invalidRenderBadFlavor = ReadEnvOrEmpty("REPULSOR_TEST_INVALID_RENDER_BAD_FLAVOR_PLUGIN_PATH");
  if (invalidSeasonNoSdk.empty() || invalidDataSourceBadFlavor.empty() || invalidRenderBadFlavor.empty()) {
    std::cerr << "Missing one or more invalid plugin env paths\n";
    return 1;
  }

  if (auto module = repulsor3d::CreateSeasonModuleFromPlugin(invalidSeasonNoSdk); module != nullptr) {
    std::cerr << "Invalid season plugin without SDK symbol unexpectedly loaded\n";
    return 2;
  }

  if (auto renderPlugin = repulsor3d::CreateRenderFeaturePluginFromPath(invalidRenderBadFlavor); renderPlugin != nullptr) {
    std::cerr << "Invalid render plugin with bad sdk flavor unexpectedly loaded\n";
    return 3;
  }

  repulsor3d::ViewerConfig cfg;
  cfg.dataSourceType = "null";
  cfg.dataSourcePluginPath = invalidDataSourceBadFlavor;
  auto source = repulsor3d::CreateDataSourceFromConfig(cfg);
  if (source == nullptr) {
    std::cerr << "Datasource factory returned null after invalid plugin fallback\n";
    return 4;
  }
  const repulsor3d::SnapshotBundle bundle = source->Read();
  if (bundle.method == "invalid_bad_flavor" || bundle.method == "plugin_template") {
    std::cerr << "Invalid datasource plugin unexpectedly active\n";
    return 5;
  }

  std::cout << "Plugin ABI negative tests passed\n";
  return 0;
}

