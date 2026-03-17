#include "repulsor3d/modules/SeasonModule.hpp"

#include <algorithm>
#include <cctype>
#include <unordered_map>
#include <utility>

#include "repulsor3d/render/SceneDescriptor.hpp"
#include "repulsor3d/render/Season2026RebuiltModelBuilder.hpp"

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

std::unique_ptr<ISeasonModule> CreateDefaultSeasonModule(const ViewerConfig& cfg) {
  if (auto module = CreateSeasonModule(cfg.sceneProfile); module != nullptr) {
    return module;
  }
  if (auto fallback = CreateSeasonModule("default"); fallback != nullptr) {
    return fallback;
  }
  return std::make_unique<Season2026RebuiltModule>();
}

}  // namespace repulsor3d

