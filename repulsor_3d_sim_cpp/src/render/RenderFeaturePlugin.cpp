#include "repulsor3d/render/RenderFeaturePlugin.hpp"

#include <filesystem>

#include "repulsor3d/plugins/PluginManifest.hpp"

#if defined(_WIN32)
#include <Windows.h>
#else
#include <dlfcn.h>
#endif

namespace repulsor3d {
namespace {

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

class DynamicRenderFeaturePluginProxy final : public IRenderFeaturePlugin {
 public:
  DynamicRenderFeaturePluginProxy(
      const LibraryHandle libraryHandle,
      IRenderFeaturePlugin* pluginInstance,
      const DestroyRenderFeaturePluginAbiFn destroyFn)
      : libraryHandle_(libraryHandle), pluginInstance_(pluginInstance), destroyFn_(destroyFn) {}

  ~DynamicRenderFeaturePluginProxy() override {
    if (pluginInstance_ != nullptr && destroyFn_ != nullptr) {
      destroyFn_(pluginInstance_);
      pluginInstance_ = nullptr;
    }
    CloseLibrary(libraryHandle_);
    libraryHandle_ = nullptr;
  }

  std::string Id() const override {
    if (pluginInstance_ == nullptr) {
      return "dynamic_render_plugin";
    }
    return pluginInstance_->Id();
  }

  std::vector<std::unique_ptr<IRenderFeature>> CreateFeatures(const ViewerConfig& cfg) const override {
    if (pluginInstance_ == nullptr) {
      return {};
    }
    return pluginInstance_->CreateFeatures(cfg);
  }

 private:
  mutable LibraryHandle libraryHandle_ = nullptr;
  mutable IRenderFeaturePlugin* pluginInstance_ = nullptr;
  DestroyRenderFeaturePluginAbiFn destroyFn_ = nullptr;
};

}  // namespace

std::unique_ptr<IRenderFeaturePlugin> CreateRenderFeaturePluginFromPath(const std::string& pluginPath) {
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

  auto* createFnRaw = ResolveSymbol(handle, "repulsor3d_create_render_feature_plugin");
  auto* destroyFnRaw = ResolveSymbol(handle, "repulsor3d_destroy_render_feature_plugin");
  auto* queryAbiVersionRaw = ResolveSymbol(handle, "repulsor3d_query_render_feature_plugin_abi_version");
  auto* queryManifestRaw = ResolveSymbol(handle, "repulsor3d_query_plugin_manifest_v1");
  if (createFnRaw == nullptr || destroyFnRaw == nullptr || queryAbiVersionRaw == nullptr) {
    CloseLibrary(handle);
    return nullptr;
  }

  const auto createFn = reinterpret_cast<CreateRenderFeaturePluginAbiFn>(createFnRaw);
  const auto destroyFn = reinterpret_cast<DestroyRenderFeaturePluginAbiFn>(destroyFnRaw);
  const auto queryAbiVersionFn = reinterpret_cast<QueryRenderFeaturePluginAbiVersionFn>(queryAbiVersionRaw);
  const auto queryManifestFn = reinterpret_cast<QueryPluginManifestV1Fn>(queryManifestRaw);
  if (queryAbiVersionFn == nullptr || queryAbiVersionFn() != kRenderFeaturePluginAbiVersion) {
    CloseLibrary(handle);
    return nullptr;
  }
  if (queryManifestFn != nullptr) {
    const PluginManifestV1* manifest = queryManifestFn();
    if (manifest == nullptr ||
        !IsPluginManifestCompatible(*manifest, PluginKind::RenderFeature, kRenderFeaturePluginAbiVersion)) {
      CloseLibrary(handle);
      return nullptr;
    }
  }

  IRenderFeaturePlugin* plugin = createFn();
  if (plugin == nullptr) {
    CloseLibrary(handle);
    return nullptr;
  }

  return std::make_unique<DynamicRenderFeaturePluginProxy>(handle, plugin, destroyFn);
}

}  // namespace repulsor3d
