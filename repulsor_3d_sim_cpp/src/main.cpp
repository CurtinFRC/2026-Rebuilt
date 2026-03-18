#include <exception>
#include <iostream>
#include <memory>

#include "repulsor3d/App.hpp"
#include "repulsor3d/Config.hpp"
#include "repulsor3d/ConfigValidation.hpp"
#include "repulsor3d/DataSourceFactory.hpp"
#include "repulsor3d/season/SeasonDefinitionRegistry.hpp"

int main() {
  try {
    repulsor3d::ViewerConfig cfg = repulsor3d::LoadConfigFromEnv();
    if (auto season = repulsor3d::CreateDefaultSeasonDefinition(cfg); season != nullptr) {
      season->ApplyDefaults(cfg);
    }
    const repulsor3d::ConfigValidationResult validation = repulsor3d::ValidateConfig(cfg);
    for (const auto& warning : validation.warnings) {
      std::cerr << "[Config warning] " << warning << "\n";
    }
    if (!validation.Ok()) {
      for (const auto& error : validation.errors) {
        std::cerr << "[Config error] " << error << "\n";
      }
      return 2;
    }

    auto source = repulsor3d::CreateDataSourceFromConfig(cfg);

    repulsor3d::ViewerApp app(cfg, std::move(source));
    return app.Run();
  } catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << "\n";
    return 2;
  }
}
