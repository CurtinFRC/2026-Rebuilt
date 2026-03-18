#include "repulsor3d/season/SeasonDefinitionRegistry.hpp"

#include <algorithm>
#include <cctype>
#include <unordered_map>

#include "repulsor3d/render/Season2026RebuiltModelBuilder.hpp"

namespace repulsor3d {
namespace {

std::string CanonicalKey(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

std::unordered_map<std::string, SeasonDefinitionFactoryFn>& Registry() {
  static std::unordered_map<std::string, SeasonDefinitionFactoryFn> registry;
  return registry;
}

class Season2026RebuiltDefinition final : public ISeasonDefinition {
 public:
  std::string Id() const override { return "2026Rebuilt"; }

  std::unique_ptr<ISceneModelBuilder> CreateSceneModelBuilder(const ViewerConfig& cfg) const override {
    return std::make_unique<Season2026RebuiltModelBuilder>(cfg);
  }

  std::string DefaultNtSchemaPath() const override {
    return "assets/schemas/2026rebuilt_nt_schema.json";
  }

  std::string DefaultCoordinateProfilePath() const override {
    return "assets/profiles/2026rebuilt_coordinates.json";
  }

  void ApplyDefaults(ViewerConfig& cfg) const override {
    if (cfg.ntSchemaPath.empty()) {
      cfg.ntSchemaPath = DefaultNtSchemaPath();
    }
    if (cfg.incomingCoordCalibrationProfilePath.empty()) {
      cfg.incomingCoordCalibrationProfilePath = DefaultCoordinateProfilePath();
    }
  }
};

void RegisterBuiltinsOnce() {
  static bool registered = false;
  if (registered) {
    return;
  }

  RegisterSeasonDefinition("2026rebuilt", [] { return std::make_unique<Season2026RebuiltDefinition>(); });
  RegisterSeasonDefinition("rebuilt2026", [] { return std::make_unique<Season2026RebuiltDefinition>(); });
  RegisterSeasonDefinition("default", [] { return std::make_unique<Season2026RebuiltDefinition>(); });
  registered = true;
}

}  // namespace

void RegisterSeasonDefinition(const std::string& seasonId, SeasonDefinitionFactoryFn factoryFn) {
  if (seasonId.empty() || !factoryFn) {
    return;
  }
  Registry()[CanonicalKey(seasonId)] = std::move(factoryFn);
}

std::unique_ptr<ISeasonDefinition> CreateSeasonDefinition(const std::string& seasonId) {
  RegisterBuiltinsOnce();
  const auto it = Registry().find(CanonicalKey(seasonId));
  if (it == Registry().end()) {
    return nullptr;
  }
  return it->second();
}

std::unique_ptr<ISeasonDefinition> CreateDefaultSeasonDefinition(const ViewerConfig& cfg) {
  if (auto season = CreateSeasonDefinition(cfg.sceneProfile); season != nullptr) {
    return season;
  }
  return CreateSeasonDefinition("default");
}

}  // namespace repulsor3d
