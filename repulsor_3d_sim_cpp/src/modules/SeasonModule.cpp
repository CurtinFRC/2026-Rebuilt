#include "repulsor3d/modules/SeasonModule.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <unordered_map>
#include <utility>

#include "repulsor3d/render/SceneDescriptor.hpp"
#include "repulsor3d/render/Season2026RebuiltModelBuilder.hpp"

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
    auto base = CreateRenderWorldAdapterFromSceneBuilder(std::make_unique<Season2026RebuiltModelBuilder>(cfg));
    if (auto descriptor = LoadSceneDescriptorForProfile(cfg); descriptor.has_value()) {
      return std::make_unique<DescriptorDecoratingRenderWorldAdapter>(std::move(base), std::move(*descriptor));
    }
    return base;
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
  if (createFnRaw == nullptr || destroyFnRaw == nullptr) {
    CloseLibrary(handle);
    return nullptr;
  }

  const auto createFn = reinterpret_cast<CreateSeasonModuleAbiFn>(createFnRaw);
  const auto destroyFn = reinterpret_cast<DestroySeasonModuleAbiFn>(destroyFnRaw);
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
