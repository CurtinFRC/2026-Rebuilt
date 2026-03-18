#pragma once

#if defined(REPULSOR_HAS_NTCORE)

#include <filesystem>
#include <string>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/nt/DefaultSchemas.hpp"

namespace repulsor3d::nt {

class TopicSchemaRegistry {
 public:
  static constexpr int kSupportedSchemaVersion = 1;

  explicit TopicSchemaRegistry(const ViewerConfig& cfg);

  const NtSchemaSet& SchemaSet() const { return schemaSet_; }
  int SchemaVersion() const { return schemaVersion_; }
  const std::string& LastError() const { return lastError_; }
  const std::string& LastConflictReport() const { return lastConflictReport_; }

  bool Refresh(double nowSeconds, bool forceReload = false);

 private:
  bool TryReadSchemaMetadata(int& outVersion, std::string& outConflictReport);
  bool LoadFromFile(bool forceLog);

  ViewerConfig cfg_;
  NtSchemaSet schemaSet_;
  int schemaVersion_ = kSupportedSchemaVersion;
  std::string lastError_;
  std::string lastConflictReport_;
  bool schemaLoadedFromFile_ = false;
  bool schemaWriteTimeValid_ = false;
  std::filesystem::file_time_type schemaWriteTime_{};
  double lastSchemaCheckS_ = 0.0;
  double schemaCheckPeriodS_ = 0.5;
};

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)
