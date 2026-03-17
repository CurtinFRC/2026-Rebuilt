#include <exception>
#include <iostream>
#include <memory>

#include "repulsor3d/App.hpp"
#include "repulsor3d/Config.hpp"
#include "repulsor3d/ConfigValidation.hpp"
#include "repulsor3d/DataSource.hpp"
#include "repulsor3d/NullDataSource.hpp"

#if defined(REPULSOR_HAS_NTCORE)
#include "repulsor3d/NtDataSource.hpp"
#endif

namespace repulsor3d {

std::unique_ptr<ISnapshotSource> CreateDataSource(const ViewerConfig& cfg) {
#if defined(REPULSOR_HAS_NTCORE)
  return std::make_unique<NtDataSource>(cfg);
#else
  (void)cfg;
  std::cerr << "NT4 backend not compiled in, using NullDataSource\n";
  return std::make_unique<NullDataSource>();
#endif
}

}  // namespace repulsor3d

int main() {
  try {
    const repulsor3d::ViewerConfig cfg = repulsor3d::LoadConfigFromEnv();
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

    auto source = repulsor3d::CreateDataSource(cfg);

    repulsor3d::ViewerApp app(cfg, std::move(source));
    return app.Run();
  } catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << "\n";
    return 2;
  }
}
