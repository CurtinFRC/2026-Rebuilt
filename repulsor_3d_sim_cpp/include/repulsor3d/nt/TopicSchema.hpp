#pragma once

#if defined(REPULSOR_HAS_NTCORE)

#include <string>
#include <vector>

#include <networktables/NetworkTableType.h>

namespace repulsor3d::nt {

struct ScalarFieldSpec {
  std::string key;
  std::string topicSuffix;
  ::nt::NetworkTableType type = ::nt::NetworkTableType::kDouble;
  double defaultDouble = 0.0;
  bool defaultBoolean = false;
  std::string defaultString;
};

struct EntityGroupSchema {
  std::string tablePrefix;
  std::string idPrefix;
  ScalarFieldSpec aliveField;
  std::vector<ScalarFieldSpec> fields;
  bool autoCaptureAdditionalScalars = true;
  std::string extraKeyPrefix = "extra.";
};

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)
