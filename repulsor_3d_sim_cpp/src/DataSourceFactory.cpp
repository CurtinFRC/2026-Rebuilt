#include "repulsor3d/DataSourceFactory.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <utility>

#include "repulsor3d/datasource/DataSourcePluginAbi.hpp"
#include "repulsor3d/plugins/PluginManifest.hpp"
#include "repulsor3d/NullDataSource.hpp"
#include "repulsor3d/ReplayDataSource.hpp"

#if defined(REPULSOR_HAS_NTCORE)
#include "repulsor3d/NtDataSource.hpp"
#endif

#if defined(_WIN32)
#include <Windows.h>
#else
#include <dlfcn.h>
#endif

namespace repulsor3d {
namespace {

std::string ToLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
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

#if defined(_WIN32)
using LibraryHandle = HMODULE;
#else
using LibraryHandle = void*;
#endif

LibraryHandle OpenLibrary(const std::string& path) {
#if defined(_WIN32)
  return LoadLibraryA(path.c_str());
#else
  return dlopen(path.c_str(), RTLD_NOW);
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

class DynamicDataSourceProxy final : public ISnapshotSource {
 public:
  DynamicDataSourceProxy(
      const LibraryHandle handle,
      ISnapshotSource* source,
      const DestroyDataSourcePluginFn destroyFn)
      : handle_(handle), source_(source), destroyFn_(destroyFn) {}

  ~DynamicDataSourceProxy() override {
    if (source_ != nullptr && destroyFn_ != nullptr) {
      destroyFn_(source_);
    }
    source_ = nullptr;
    CloseLibrary(handle_);
    handle_ = nullptr;
  }

  SnapshotBundle Read() override {
    if (source_ == nullptr) {
      return SnapshotBundle{};
    }
    return source_->Read();
  }

 private:
  mutable LibraryHandle handle_ = nullptr;
  mutable ISnapshotSource* source_ = nullptr;
  DestroyDataSourcePluginFn destroyFn_ = nullptr;
};

std::unique_ptr<ISnapshotSource> CreateDataSourceFromPlugin(const std::string& pluginPath, const ViewerConfig& cfg) {
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

  const auto createFn = reinterpret_cast<CreateDataSourcePluginFn>(
      ResolveSymbol(handle, "repulsor3d_create_data_source_plugin"));
  const auto destroyFn = reinterpret_cast<DestroyDataSourcePluginFn>(
      ResolveSymbol(handle, "repulsor3d_destroy_data_source_plugin"));
  const auto queryAbiFn = reinterpret_cast<QueryDataSourcePluginAbiVersionFn>(
      ResolveSymbol(handle, "repulsor3d_query_data_source_plugin_abi_version"));
  const auto queryManifestFn = reinterpret_cast<QueryPluginManifestV1Fn>(
      ResolveSymbol(handle, "repulsor3d_query_plugin_manifest_v1"));
  if (createFn == nullptr || destroyFn == nullptr || queryAbiFn == nullptr) {
    CloseLibrary(handle);
    return nullptr;
  }

  const int abiVersion = queryAbiFn();
  if (abiVersion != kDataSourcePluginAbiVersion) {
    CloseLibrary(handle);
    return nullptr;
  }
  if (queryManifestFn != nullptr) {
    const PluginManifestV1* manifest = queryManifestFn();
    if (manifest == nullptr ||
        !IsPluginManifestCompatible(*manifest, PluginKind::DataSource, abiVersion)) {
      CloseLibrary(handle);
      return nullptr;
    }
    const std::uint64_t requiredCaps = ParseRequiredCapabilityFlags("DATASOURCE_PLUGIN_REQUIRED_CAPS");
    if (requiredCaps != 0ULL && !HasRequiredCapabilities(*manifest, requiredCaps)) {
      CloseLibrary(handle);
      return nullptr;
    }
  }

  ISnapshotSource* source = createFn(cfg);
  if (source == nullptr) {
    CloseLibrary(handle);
    return nullptr;
  }

  return std::make_unique<DynamicDataSourceProxy>(handle, source, destroyFn);
}

}  // namespace

void DataSourceRegistry::Register(std::string name, Creator creator) {
  creators_[ToLower(std::move(name))] = std::move(creator);
}

std::unique_ptr<ISnapshotSource> DataSourceRegistry::Create(const std::string& name, const ViewerConfig& cfg) const {
  const auto it = creators_.find(ToLower(name));
  if (it == creators_.end()) {
    return nullptr;
  }
  return it->second(cfg);
}

bool DataSourceRegistry::Contains(const std::string& name) const {
  return creators_.contains(ToLower(name));
}

DataSourceRegistry CreateDefaultDataSourceRegistry() {
  DataSourceRegistry registry;

  registry.Register("null", [](const ViewerConfig&) { return std::make_unique<NullDataSource>(); });
  registry.Register("replay", [](const ViewerConfig& cfg) {
    return std::make_unique<ReplayDataSource>(cfg.replaySnapshotPath, cfg.replayLoop);
  });

#if defined(REPULSOR_HAS_NTCORE)
  registry.Register("nt", [](const ViewerConfig& cfg) { return std::make_unique<NtDataSource>(cfg); });
#else
  registry.Register("nt", [](const ViewerConfig&) {
    std::cerr << "NT4 backend not compiled in, using NullDataSource\n";
    return std::make_unique<NullDataSource>();
  });
#endif

  return registry;
}

std::unique_ptr<ISnapshotSource> CreateDataSourceFromConfig(const ViewerConfig& cfg) {
  const DataSourceRegistry registry = CreateDefaultDataSourceRegistry();
  const std::string mode = ToLower(cfg.dataSourceType);

  if (!cfg.dataSourcePluginPath.empty()) {
    if (auto plugin = CreateDataSourceFromPlugin(cfg.dataSourcePluginPath, cfg); plugin != nullptr) {
      std::cerr << "[DataSourceFactory] using plugin datasource\n";
      return plugin;
    }
    std::cerr << "[DataSourceFactory] failed to load datasource plugin: " << cfg.dataSourcePluginPath << "\n";
  }

  if (mode == "auto") {
    if (!cfg.replaySnapshotPath.empty()) {
      if (auto replay = registry.Create("replay", cfg); replay != nullptr) {
        std::cerr << "[DataSourceFactory] using replay datasource\n";
        return replay;
      }
    }
    if (auto nt = registry.Create("nt", cfg); nt != nullptr) {
      std::cerr << "[DataSourceFactory] using nt datasource\n";
      return nt;
    }
    return std::make_unique<NullDataSource>();
  }

  if (auto requested = registry.Create(mode, cfg); requested != nullptr) {
    return requested;
  }

  std::cerr << "[DataSourceFactory] unknown datasource '" << cfg.dataSourceType << "', using null\n";
  return std::make_unique<NullDataSource>();
}

}  // namespace repulsor3d
