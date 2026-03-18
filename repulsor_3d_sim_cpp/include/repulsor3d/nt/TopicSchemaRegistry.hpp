#pragma once

#if defined(REPULSOR_HAS_NTCORE)

#include <filesystem>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/nt/DefaultSchemas.hpp"

namespace repulsor3d::nt {

class TopicSchemaRegistry {
 public:
  explicit TopicSchemaRegistry(const ViewerConfig& cfg);

  const NtSchemaSet& SchemaSet() const { return schemaSet_; }

  bool Refresh(double nowSeconds, bool forceReload = false);

 private:
  bool LoadFromFile(bool forceLog);

  ViewerConfig cfg_;
  NtSchemaSet schemaSet_;
  bool schemaLoadedFromFile_ = false;
  bool schemaWriteTimeValid_ = false;
  std::filesystem::file_time_type schemaWriteTime_{};
  double lastSchemaCheckS_ = 0.0;
  double schemaCheckPeriodS_ = 0.5;
};

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)
