#include "repulsor3d/render/SceneDescriptor.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

namespace repulsor3d {
namespace {

bool TryReadBool(const nlohmann::json& node, const char* key, std::optional<bool>& out) {
  if (!node.contains(key) || !node[key].is_boolean()) {
    return false;
  }
  out = node[key].get<bool>();
  return true;
}

glm::vec4 ParseColor(const nlohmann::json& colorNode, const glm::vec4& fallback) {
  if (!colorNode.is_array() || colorNode.size() < 3) {
    return fallback;
  }

  const float r = colorNode[0].is_number() ? colorNode[0].get<float>() : fallback.r;
  const float g = colorNode[1].is_number() ? colorNode[1].get<float>() : fallback.g;
  const float b = colorNode[2].is_number() ? colorNode[2].get<float>() : fallback.b;
  const float a = (colorNode.size() > 3 && colorNode[3].is_number()) ? colorNode[3].get<float>() : fallback.a;
  return glm::vec4{r, g, b, a};
}

OverlayAnchor ParseAnchor(const nlohmann::json& anchorNode, const OverlayAnchor fallback) {
  if (!anchorNode.is_string()) {
    return fallback;
  }

  std::string value = anchorNode.get<std::string>();
  std::transform(
      value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (value == "topleft" || value == "top_left") {
    return OverlayAnchor::TopLeft;
  }
  if (value == "topright" || value == "top_right") {
    return OverlayAnchor::TopRight;
  }
  if (value == "bottomleft" || value == "bottom_left") {
    return OverlayAnchor::BottomLeft;
  }
  if (value == "bottomright" || value == "bottom_right") {
    return OverlayAnchor::BottomRight;
  }
  return fallback;
}

}  // namespace

std::string CanonicalSceneProfileKey(const std::string& sceneProfile) {
  std::string out;
  out.reserve(sceneProfile.size());

  for (const char c : sceneProfile) {
    if (std::isalnum(static_cast<unsigned char>(c)) != 0) {
      out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
  }

  return out.empty() ? std::string{"default"} : out;
}

std::optional<SceneDescriptor> LoadSceneDescriptorFromFile(const std::string& path) {
  std::ifstream in(path);
  if (!in.is_open()) {
    return std::nullopt;
  }

  nlohmann::json root;
  try {
    in >> root;
  } catch (...) {
    return std::nullopt;
  }

  SceneDescriptor descriptor;
  TryReadBool(root, "drawFieldImage", descriptor.drawFieldImage);
  TryReadBool(root, "drawGrid", descriptor.drawGrid);
  TryReadBool(root, "drawAxes", descriptor.drawAxes);

  if (root.contains("overlay") && root["overlay"].is_array()) {
    for (const auto& item : root["overlay"]) {
      if (!item.is_object()) {
        continue;
      }
      if (!item.contains("text") || !item["text"].is_string()) {
        continue;
      }
      OverlayLine line;
      line.text = item["text"].get<std::string>();
      if (item.contains("color")) {
        line.color = ParseColor(item["color"], line.color);
      }
      if (item.contains("anchor")) {
        line.anchor = ParseAnchor(item["anchor"], line.anchor);
      }
      if (item.contains("marginX") && item["marginX"].is_number()) {
        line.marginX = item["marginX"].get<float>();
      }
      if (item.contains("marginY") && item["marginY"].is_number()) {
        line.marginY = item["marginY"].get<float>();
      }
      descriptor.staticOverlayLines.push_back(std::move(line));
    }
  }

  return descriptor;
}

std::optional<SceneDescriptor> LoadSceneDescriptorForProfile(const ViewerConfig& cfg) {
  std::vector<std::string> candidates;
  candidates.reserve(3);

  if (!cfg.sceneDescriptorPath.empty()) {
    candidates.push_back(cfg.sceneDescriptorPath);
  }

  const std::string profileKey = CanonicalSceneProfileKey(cfg.sceneProfile);
  candidates.push_back("assets/scenes/" + profileKey + ".json");
  candidates.push_back("assets/scenes/default.json");

  for (const auto& candidate : candidates) {
    std::error_code ec;
    if (!std::filesystem::exists(candidate, ec) || ec) {
      continue;
    }

    if (auto descriptor = LoadSceneDescriptorFromFile(candidate); descriptor.has_value()) {
      return descriptor;
    }
  }

  return std::nullopt;
}

}  // namespace repulsor3d
