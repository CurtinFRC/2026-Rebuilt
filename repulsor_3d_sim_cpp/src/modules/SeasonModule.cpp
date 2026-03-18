#include "repulsor3d/modules/SeasonModule.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <unordered_map>
#include <utility>

#include "repulsor3d/render/Season2026RebuiltModelBuilder.hpp"
#include "repulsor3d/plugins/PluginManifest.hpp"
#include "repulsor3d/plugins/PluginSdk.hpp"
#include "repulsor3d/season/SeasonDefinitionRegistry.hpp"

#if defined(_WIN32)
#include <Windows.h>
#else
#include <dlfcn.h>
#endif

namespace repulsor3d {
namespace {

std::string CanonicalKey(const std::string& value) {
  std::string out;
  out.reserve(value.size());
  for (const char c : value) {
    if (std::isalnum(static_cast<unsigned char>(c)) != 0) {
      out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
  }
  return out.empty() ? std::string{"default"} : out;
}

std::uint64_t ParseRequiredCapabilityFlags(const char* envName) {
  const char* value = std::getenv(envName);
  if (value == nullptr || *value == '\0') {
    return 0ULL;
  }
  try {
    return std::stoull(value, nullptr, 0);
  } catch (...) {
    return 0ULL;
  }
}

std::unordered_map<std::string, SeasonModuleFactoryFn>& Registry() {
  static std::unordered_map<std::string, SeasonModuleFactoryFn> registry;
  return registry;
}

#if defined(_WIN32)
using LibraryHandle = HMODULE;
#else
using LibraryHandle = void*;
#endif

LibraryHandle OpenLibrary(const std::string& pluginPath) {
#if defined(_WIN32)
  return LoadLibraryA(pluginPath.c_str());
#else
  return dlopen(pluginPath.c_str(), RTLD_NOW);
#endif
}

void CloseLibrary(const LibraryHandle handle) {
  if (handle == nullptr) {
    return;
  }
#if defined(_WIN32)
  FreeLibrary(handle);
#else
  dlclose(handle);
#endif
}

void* ResolveSymbol(const LibraryHandle handle, const char* symbolName) {
  if (handle == nullptr || symbolName == nullptr) {
    return nullptr;
  }
#if defined(_WIN32)
  return reinterpret_cast<void*>(GetProcAddress(handle, symbolName));
#else
  return dlsym(handle, symbolName);
#endif
}

class DynamicSeasonModuleProxy final : public ISeasonModule {
 public:
  DynamicSeasonModuleProxy(
      const LibraryHandle libraryHandle,
      ISeasonModule* moduleInstance,
      DestroySeasonModuleAbiFn destroyFn)
      : libraryHandle_(libraryHandle), moduleInstance_(moduleInstance), destroyFn_(destroyFn) {}

  ~DynamicSeasonModuleProxy() override {
    if (moduleInstance_ != nullptr && destroyFn_ != nullptr) {
      destroyFn_(moduleInstance_);
      moduleInstance_ = nullptr;
    }
    CloseLibrary(libraryHandle_);
    libraryHandle_ = nullptr;
  }

  std::string Id() const override {
    if (moduleInstance_ == nullptr) {
      return "dynamic_plugin";
    }
    return moduleInstance_->Id();
  }

  std::unique_ptr<IRenderWorldAdapter> CreateWorldAdapter(const ViewerConfig& cfg) const override {
    if (moduleInstance_ == nullptr) {
      return nullptr;
    }
    return moduleInstance_->CreateWorldAdapter(cfg);
  }

 private:
  mutable LibraryHandle libraryHandle_ = nullptr;
  mutable ISeasonModule* moduleInstance_ = nullptr;
  DestroySeasonModuleAbiFn destroyFn_ = nullptr;
};

class Season2026RebuiltModule final : public ISeasonModule {
 public:
  std::string Id() const override { return "2026rebuilt"; }

  std::unique_ptr<IRenderWorldAdapter> CreateWorldAdapter(const ViewerConfig& cfg) const override {
    if (auto season = CreateSeasonDefinition("2026rebuilt"); season != nullptr) {
      return season->CreateWorldAdapter(cfg);
    }
    return CreateRenderWorldAdapterFromSceneBuilder(std::make_unique<Season2026RebuiltModelBuilder>(cfg));
  }
};

void RegisterBuiltinsOnce() {
  static bool registered = false;
  if (registered) {
    return;
  }

  RegisterSeasonModule("2026rebuilt", [] { return std::make_unique<Season2026RebuiltModule>(); });
  RegisterSeasonModule("rebuilt2026", [] { return std::make_unique<Season2026RebuiltModule>(); });
  RegisterSeasonModule("default", [] { return std::make_unique<Season2026RebuiltModule>(); });

  registered = true;
}

}  // namespace

void RegisterSeasonModule(const std::string& moduleId, SeasonModuleFactoryFn factoryFn) {
  if (moduleId.empty() || !factoryFn) {
    return;
  }
  Registry()[CanonicalKey(moduleId)] = std::move(factoryFn);
}

std::unique_ptr<ISeasonModule> CreateSeasonModule(const std::string& moduleId) {
  RegisterBuiltinsOnce();

  const std::string key = CanonicalKey(moduleId);
  const auto it = Registry().find(key);
  if (it != Registry().end()) {
    return it->second();
  }
  return nullptr;
}

std::unique_ptr<ISeasonModule> CreateSeasonModuleFromPlugin(const std::string& pluginPath) {
  if (pluginPath.empty()) {
    return nullptr;
  }

  std::error_code ec;
  if (!std::filesystem::exists(pluginPath, ec) || ec) {
    return nullptr;
  }

  const LibraryHandle handle = OpenLibrary(pluginPath);
  if (handle == nullptr) {
    return nullptr;
  }

  auto* createFnRaw = ResolveSymbol(handle, "repulsor3d_create_season_module");
  auto* destroyFnRaw = ResolveSymbol(handle, "repulsor3d_destroy_season_module");
  auto* queryAbiVersionRaw = ResolveSymbol(handle, "repulsor3d_query_season_module_abi_version");
  auto* queryManifestRaw = ResolveSymbol(handle, "repulsor3d_query_plugin_manifest_v1");
  auto* querySdkRaw = ResolveSymbol(handle, "repulsor3d_query_plugin_sdk_info_v1");
  if (createFnRaw == nullptr || destroyFnRaw == nullptr || queryAbiVersionRaw == nullptr) {
    CloseLibrary(handle);
    return nullptr;
  }

  const auto createFn = reinterpret_cast<CreateSeasonModuleAbiFn>(createFnRaw);
  const auto destroyFn = reinterpret_cast<DestroySeasonModuleAbiFn>(destroyFnRaw);
  const auto queryAbiVersionFn = reinterpret_cast<QuerySeasonModuleAbiVersionFn>(queryAbiVersionRaw);
  const auto queryManifestFn = reinterpret_cast<QueryPluginManifestV1Fn>(queryManifestRaw);
  const auto querySdkFn = reinterpret_cast<QueryPluginSdkInfoV1Fn>(querySdkRaw);
  if (queryAbiVersionFn == nullptr || queryAbiVersionFn() != kSeasonModuleAbiVersion) {
    CloseLibrary(handle);
    return nullptr;
  }
  if (querySdkFn != nullptr) {
    const PluginSdkInfoV1* sdkInfo = querySdkFn();
    if (sdkInfo == nullptr || !IsPluginSdkCompatible(*sdkInfo, kPluginSdkVersion)) {
      CloseLibrary(handle);
      return nullptr;
    }
  }
  if (queryManifestFn != nullptr) {
    const PluginManifestV1* manifest = queryManifestFn();
    if (manifest == nullptr ||
        !IsPluginManifestCompatible(*manifest, PluginKind::SeasonModule, kSeasonModuleAbiVersion)) {
      CloseLibrary(handle);
      return nullptr;
    }
    const std::uint64_t requiredCaps = ParseRequiredCapabilityFlags("SEASON_PLUGIN_REQUIRED_CAPS");
    if (requiredCaps != 0ULL && !HasRequiredCapabilities(*manifest, requiredCaps)) {
      CloseLibrary(handle);
      return nullptr;
    }
  }

  ISeasonModule* module = createFn();
  if (module == nullptr) {
    CloseLibrary(handle);
    return nullptr;
  }

  return std::make_unique<DynamicSeasonModuleProxy>(handle, module, destroyFn);
}

std::unique_ptr<ISeasonModule> CreateDefaultSeasonModule(const ViewerConfig& cfg) {
  if (!cfg.seasonModulePluginPath.empty()) {
    if (auto pluginModule = CreateSeasonModuleFromPlugin(cfg.seasonModulePluginPath); pluginModule != nullptr) {
      return pluginModule;
    }
  }

  if (auto module = CreateSeasonModule(cfg.sceneProfile); module != nullptr) {
    return module;
  }
  if (auto fallback = CreateSeasonModule("default"); fallback != nullptr) {
    return fallback;
  }
  return std::make_unique<Season2026RebuiltModule>();
}

}  // namespace repulsor3d
