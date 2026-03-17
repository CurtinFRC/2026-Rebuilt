#include "repulsor3d/render/SceneModelBuilderFactory.hpp"

#include <algorithm>
#include <cctype>
#include <unordered_map>

#include "repulsor3d/render/Season2026RebuiltModelBuilder.hpp"

namespace repulsor3d {
namespace {

std::string ToLower(const std::string& s) {
  std::string out = s;
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}

std::unordered_map<std::string, SceneModelBuilderFactoryFn>& Registry() {
  static std::unordered_map<std::string, SceneModelBuilderFactoryFn> registry;
  return registry;
}

void RegisterBuiltinsOnce() {
  static bool registered = false;
  if (registered) {
    return;
  }

  RegisterSceneModelBuilder("2026rebuilt", [](const ViewerConfig& cfg) {
    return std::make_unique<Season2026RebuiltModelBuilder>(cfg);
  });
  RegisterSceneModelBuilder("rebuilt2026", [](const ViewerConfig& cfg) {
    return std::make_unique<Season2026RebuiltModelBuilder>(cfg);
  });
  RegisterSceneModelBuilder("default", [](const ViewerConfig& cfg) {
    return std::make_unique<Season2026RebuiltModelBuilder>(cfg);
  });

  registered = true;
}

}  // namespace

void RegisterSceneModelBuilder(const std::string& sceneProfile, SceneModelBuilderFactoryFn fn) {
  if (sceneProfile.empty() || !fn) {
    return;
  }
  Registry()[ToLower(sceneProfile)] = std::move(fn);
}

std::unique_ptr<ISceneModelBuilder> CreateSceneModelBuilder(const std::string& sceneProfile, const ViewerConfig& cfg) {
  RegisterBuiltinsOnce();

  const std::string key = ToLower(sceneProfile);
  const auto it = Registry().find(key);
  if (it != Registry().end()) {
    return it->second(cfg);
  }

  // Fallback to current season so renderer always has a concrete model builder.
  return std::make_unique<Season2026RebuiltModelBuilder>(cfg);
}

std::unique_ptr<ISceneModelBuilder> CreateDefaultSceneModelBuilder(const ViewerConfig& cfg) {
  return CreateSceneModelBuilder(cfg.sceneProfile, cfg);
}

}  // namespace repulsor3d
