#include "repulsor3d/render/SceneModelBuilderFactory.hpp"

#include <algorithm>
#include <cctype>

#include "repulsor3d/render/Season2026RebuiltModelBuilder.hpp"

namespace repulsor3d {
namespace {

std::string ToLower(const std::string& s) {
  std::string out = s;
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}

}  // namespace

std::unique_ptr<ISceneModelBuilder> CreateSceneModelBuilder(const std::string& sceneProfile, const ViewerConfig& cfg) {
  const std::string key = ToLower(sceneProfile);
  if (key == "2026rebuilt" || key == "rebuilt2026" || key == "default") {
    return std::make_unique<Season2026RebuiltModelBuilder>(cfg);
  }

  // Fallback to current season so renderer always has a concrete model builder.
  return std::make_unique<Season2026RebuiltModelBuilder>(cfg);
}

std::unique_ptr<ISceneModelBuilder> CreateDefaultSceneModelBuilder(const ViewerConfig& cfg) {
  return CreateSceneModelBuilder(cfg.sceneProfile, cfg);
}

}  // namespace repulsor3d
