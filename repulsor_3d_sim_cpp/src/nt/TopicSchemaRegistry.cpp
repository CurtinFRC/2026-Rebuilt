#if defined(REPULSOR_HAS_NTCORE)

#include "repulsor3d/nt/TopicSchemaRegistry.hpp"

#include <fstream>
#include <iostream>
#include <unordered_set>

#include <nlohmann/json.hpp>
#include "repulsor3d/nt/SchemaFileLoader.hpp"

namespace repulsor3d::nt {

TopicSchemaRegistry::TopicSchemaRegistry(const ViewerConfig& cfg)
    : cfg_(cfg), schemaSet_(MakeDefaultSchemaSet(cfg_)) {
  LoadFromFile(false);
}

bool TopicSchemaRegistry::TryReadSchemaMetadata(int& outVersion, std::string& outConflictReport) {
  outVersion = kSupportedSchemaVersion;
  outConflictReport.clear();

  if (cfg_.ntSchemaPath.empty()) {
    return false;
  }

  std::ifstream in(cfg_.ntSchemaPath);
  if (!in.is_open()) {
    return false;
  }

  nlohmann::json root;
  try {
    in >> root;
  } catch (...) {
    return false;
  }
  if (!root.is_object()) {
    return false;
  }

  outVersion = root.value("schemaVersion", kSupportedSchemaVersion);

  if (root.contains("channels") && root["channels"].is_array()) {
    std::unordered_set<std::string> seenChannels;
    for (const auto& channelNode : root["channels"]) {
      if (!channelNode.is_object()) {
        continue;
      }
      const std::string channel = channelNode.value("channel", std::string{});
      if (channel.empty()) {
        continue;
      }
      if (!seenChannels.insert(channel).second) {
        if (!outConflictReport.empty()) {
          outConflictReport += "; ";
        }
        outConflictReport += "duplicate channel '" + channel + "'";
      }
    }
  }
  return true;
}

bool TopicSchemaRegistry::LoadFromFile(const bool forceLog) {
  if (cfg_.ntSchemaPath.empty()) {
    return false;
  }

  int candidateVersion = kSupportedSchemaVersion;
  std::string conflictReport;
  TryReadSchemaMetadata(candidateVersion, conflictReport);
  lastConflictReport_ = std::move(conflictReport);
  if (candidateVersion > kSupportedSchemaVersion) {
    lastError_ = "unsupported schemaVersion " + std::to_string(candidateVersion) +
                 " (host supports up to " + std::to_string(kSupportedSchemaVersion) + ")";
    if (forceLog) {
      std::cerr << "[TopicSchemaRegistry] " << lastError_ << "\n";
    }
    return false;
  }
  schemaVersion_ = candidateVersion;

  NtSchemaSet candidate = MakeDefaultSchemaSet(cfg_);
  std::string error;
  if (!LoadSchemaSetFromFile(cfg_.ntSchemaPath, candidate, &error)) {
    lastError_ = error;
    if (forceLog) {
      std::cerr << "[TopicSchemaRegistry] failed to load schema file '" << cfg_.ntSchemaPath << "': " << error << "\n";
    }
    return false;
  }
  lastError_.clear();

  schemaSet_ = std::move(candidate);
  schemaLoadedFromFile_ = true;
  if (forceLog && !lastConflictReport_.empty()) {
    std::cerr << "[TopicSchemaRegistry] schema conflict report: " << lastConflictReport_ << "\n";
  }
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
