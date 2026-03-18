#pragma once

#include <cstdint>

namespace repulsor3d {

enum class PluginKind : std::int32_t {
  Unknown = 0,
  SeasonModule = 1,
  DataSource = 2,
  RenderFeature = 3,
};

enum class PluginCapability : std::uint64_t {
  None = 0ULL,
  HotReloadSafe = 1ULL << 0U,
  DiagnosticsAware = 1ULL << 1U,
  SceneDescriptorAware = 1ULL << 2U,
  DynamicEntitiesAware = 1ULL << 3U,
};

constexpr inline std::uint64_t ToCapabilityBits(const PluginCapability capability) {
  return static_cast<std::uint64_t>(capability);
}

struct PluginManifestV1 {
  std::int32_t structSize = static_cast<std::int32_t>(sizeof(PluginManifestV1));
  std::int32_t pluginKind = static_cast<std::int32_t>(PluginKind::Unknown);
  std::int32_t abiVersion = 0;
  std::int32_t minHostAbiVersion = 0;
  std::int32_t maxHostAbiVersion = 0;
  const char* pluginId = nullptr;
  const char* pluginVersion = nullptr;
  const char* buildSignature = nullptr;      // e.g. git hash or sha256 string
  std::uint64_t capabilityFlags = 0ULL;      // bitmask of PluginCapability
};

using QueryPluginManifestV1Fn = const PluginManifestV1* (*)();

inline bool IsPluginManifestCompatible(
    const PluginManifestV1& manifest,
    const PluginKind expectedKind,
    const int hostAbiVersion) {
  if (manifest.structSize < static_cast<std::int32_t>(sizeof(PluginManifestV1))) {
    return false;
  }
  if (manifest.pluginKind != static_cast<std::int32_t>(expectedKind)) {
    return false;
  }
  if (manifest.abiVersion <= 0) {
    return false;
  }
  if (manifest.minHostAbiVersion > hostAbiVersion) {
    return false;
  }
  if (manifest.maxHostAbiVersion > 0 && hostAbiVersion > manifest.maxHostAbiVersion) {
    return false;
  }
  return true;
}

inline bool HasRequiredCapabilities(const PluginManifestV1& manifest, const std::uint64_t requiredFlags) {
  return (manifest.capabilityFlags & requiredFlags) == requiredFlags;
}

}  // namespace repulsor3d
