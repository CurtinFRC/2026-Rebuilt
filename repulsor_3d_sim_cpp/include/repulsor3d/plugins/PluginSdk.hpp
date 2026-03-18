#pragma once

#include <cstdint>
#include <string_view>

namespace repulsor3d {

inline constexpr int kPluginSdkVersion = 1;

struct PluginSdkInfoV1 {
  std::int32_t structSize = static_cast<std::int32_t>(sizeof(PluginSdkInfoV1));
  std::int32_t sdkVersion = kPluginSdkVersion;
  std::int32_t minHostSdkVersion = kPluginSdkVersion;
  std::int32_t maxHostSdkVersion = kPluginSdkVersion;
  const char* sdkFlavor = "repulsor3d";
};

using QueryPluginSdkInfoV1Fn = const PluginSdkInfoV1* (*)();

inline bool IsPluginSdkCompatible(const PluginSdkInfoV1& sdkInfo, const int hostSdkVersion) {
  if (sdkInfo.structSize < static_cast<std::int32_t>(sizeof(PluginSdkInfoV1))) {
    return false;
  }
  if (sdkInfo.sdkVersion <= 0) {
    return false;
  }
  if (sdkInfo.minHostSdkVersion > hostSdkVersion) {
    return false;
  }
  if (sdkInfo.maxHostSdkVersion > 0 && hostSdkVersion > sdkInfo.maxHostSdkVersion) {
    return false;
  }
  if (sdkInfo.sdkFlavor == nullptr || std::string_view(sdkInfo.sdkFlavor) != std::string_view("repulsor3d")) {
    return false;
  }
  return true;
}

}  // namespace repulsor3d
