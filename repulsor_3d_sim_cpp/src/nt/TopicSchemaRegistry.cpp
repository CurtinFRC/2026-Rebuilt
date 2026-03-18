#if defined(REPULSOR_HAS_NTCORE)

#include "repulsor3d/nt/TopicSchemaRegistry.hpp"

#include <iostream>

#include "repulsor3d/nt/SchemaFileLoader.hpp"

namespace repulsor3d::nt {

TopicSchemaRegistry::TopicSchemaRegistry(const ViewerConfig& cfg)
    : cfg_(cfg), schemaSet_(MakeDefaultSchemaSet(cfg_)) {
  LoadFromFile(false);
}

bool TopicSchemaRegistry::LoadFromFile(const bool forceLog) {
  if (cfg_.ntSchemaPath.empty()) {
    return false;
  }

  NtSchemaSet candidate = MakeDefaultSchemaSet(cfg_);
  std::string error;
  if (!LoadSchemaSetFromFile(cfg_.ntSchemaPath, candidate, &error)) {
    if (forceLog) {
      std::cerr << "[TopicSchemaRegistry] failed to load schema file '" << cfg_.ntSchemaPath << "': " << error << "\n";
    }
    return false;
  }

  schemaSet_ = std::move(candidate);
  schemaLoadedFromFile_ = true;
  return true;
}

bool TopicSchemaRegistry::Refresh(const double nowSeconds, const bool forceReload) {
  if (cfg_.ntSchemaPath.empty()) {
    return false;
  }
  if (!cfg_.hotReloadNtSchema && schemaLoadedFromFile_ && !forceReload) {
    return false;
  }
  if (!forceReload && nowSeconds - lastSchemaCheckS_ < schemaCheckPeriodS_) {
    return false;
  }
  lastSchemaCheckS_ = nowSeconds;

  std::error_code ec;
  const auto stamp = std::filesystem::last_write_time(cfg_.ntSchemaPath, ec);
  if (ec) {
    return false;
  }

  if (forceReload || !schemaWriteTimeValid_ || stamp != schemaWriteTime_) {
    schemaWriteTime_ = stamp;
    schemaWriteTimeValid_ = true;
    return LoadFromFile(true);
  }
  return false;
}

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)
