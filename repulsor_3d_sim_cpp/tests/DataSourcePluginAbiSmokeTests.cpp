#include <cstdlib>
#include <iostream>
#include <string>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/DataSourceFactory.hpp"

int main() {
  const char* path = std::getenv("REPULSOR_TEST_DS_PLUGIN_PATH");
  if (path == nullptr || *path == '\0') {
    std::cerr << "REPULSOR_TEST_DS_PLUGIN_PATH not set\n";
    return 1;
  }

  repulsor3d::ViewerConfig cfg;
  cfg.dataSourceType = "auto";
  cfg.dataSourcePluginPath = path;

  auto source = repulsor3d::CreateDataSourceFromConfig(cfg);
  if (source == nullptr) {
    std::cerr << "Failed to create plugin datasource\n";
    return 2;
  }

  const repulsor3d::SnapshotBundle bundle = source->Read();
  if (bundle.method != "plugin_template") {
    std::cerr << "Unexpected datasource plugin output method: " << bundle.method << "\n";
    return 3;
  }

  std::cout << "Datasource plugin ABI smoke test passed\n";
  return 0;
}
