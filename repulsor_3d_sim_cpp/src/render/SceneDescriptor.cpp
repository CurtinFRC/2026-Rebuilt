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

bool ValidateTopLevelType(const nlohmann::json& root, const char* key, const nlohmann::json::value_t expectedType) {
  if (!root.contains(key)) {
    return true;
  }
  return root[key].type() == expectedType;
}

bool ValidateItemObjectArray(const nlohmann::json& root, const char* key) {
  if (!root.contains(key)) {
    return true;
  }
  if (!root[key].is_array()) {
    return false;
  }
  for (const auto& item : root[key]) {
    if (!item.is_object()) {
      return false;
    }
  }
  return true;
}

bool TryReadBool(const nlohmann::json& node, const char* key, std::optional<bool>& out) {
  if (!node.contains(key) || !node[key].is_boolean()) {
    return false;
  }
  out = node[key].get<bool>();
  return true;
}

glm::vec3 ParseVec3(const nlohmann::json& vecNode, const glm::vec3& fallback) {
  if (!vecNode.is_array() || vecNode.size() < 3) {
    return fallback;
  }

  const float x = vecNode[0].is_number() ? vecNode[0].get<float>() : fallback.x;
  const float y = vecNode[1].is_number() ? vecNode[1].get<float>() : fallback.y;
  const float z = vecNode[2].is_number() ? vecNode[2].get<float>() : fallback.z;
  return glm::vec3{x, y, z};
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

RenderPass ParseRenderPass(const nlohmann::json& passNode, const RenderPass fallback) {
  if (!passNode.is_string()) {
    return fallback;
  }

  std::string value = passNode.get<std::string>();
  std::transform(
      value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (value == "background") {
    return RenderPass::Background;
  }
  if (value == "opaque") {
    return RenderPass::Opaque;
  }
  if (value == "transparent") {
    return RenderPass::Transparent;
  }
  if (value == "overlay") {
    return RenderPass::Overlay;
  }
  return fallback;
}

std::string ParseString(const nlohmann::json& node, const char* key, const std::string& fallback) {
  if (!node.contains(key) || !node[key].is_string()) {
    return fallback;
  }
  return node[key].get<std::string>();
}

Transform3D ParseTransformNode(const nlohmann::json& transformNode, const Transform3D& fallback) {
  if (!transformNode.is_object()) {
    return fallback;
  }

  Transform3D out = fallback;
  if (transformNode.contains("position")) {
    out.position = ParseVec3(transformNode["position"], out.position);
  }
  if (transformNode.contains("rotationDeg")) {
    out.rotationDeg = ParseVec3(transformNode["rotationDeg"], out.rotationDeg);
  }
  if (transformNode.contains("scale")) {
    out.scale = ParseVec3(transformNode["scale"], out.scale);
  }
  return out;
}

SceneDescriptor::DynamicEntityBinding ParseDynamicEntityBinding(const nlohmann::json& node) {
  SceneDescriptor::DynamicEntityBinding out;
  out.channel = ParseString(node, "channel", out.channel);
  out.entityType = ParseString(node, "entityType", out.entityType);
  out.idPrefix = ParseString(node, "idPrefix", out.idPrefix);
  out.pass = ParseRenderPass(node.value("pass", nlohmann::json("opaque")), out.pass);
  out.xKey = ParseString(node, "xKey", out.xKey);
  out.yKey = ParseString(node, "yKey", out.yKey);
  out.zKey = ParseString(node, "zKey", out.zKey);
  out.yawDegKey = ParseString(node, "yawDegKey", out.yawDegKey);
  out.textKey = ParseString(node, "textKey", out.textKey);
  out.assetPath = ParseString(node, "assetPath", out.assetPath);
  if (node.contains("defaultRadius") && node["defaultRadius"].is_number()) {
    out.defaultRadius = node["defaultRadius"].get<float>();
  }
  if (node.contains("defaultSize")) {
    out.defaultSize = ParseVec3(node["defaultSize"], out.defaultSize);
  }
  if (node.contains("defaultScale")) {
    out.defaultScale = ParseVec3(node["defaultScale"], out.defaultScale);
  }
  if (node.contains("color")) {
    out.color = ParseColor(node["color"], out.color);
  }
  if (node.contains("wireframe") && node["wireframe"].is_boolean()) {
    out.wireframe = node["wireframe"].get<bool>();
  }
  if (node.contains("useAssetColor") && node["useAssetColor"].is_boolean()) {
    out.useAssetColor = node["useAssetColor"].get<bool>();
  }
  if (node.contains("roughnessOverride") && node["roughnessOverride"].is_number()) {
    out.roughnessOverride = node["roughnessOverride"].get<float>();
  }
  if (node.contains("metallicOverride") && node["metallicOverride"].is_number()) {
    out.metallicOverride = node["metallicOverride"].get<float>();
  }
  if (node.contains("normalStrength") && node["normalStrength"].is_number()) {
    out.normalStrength = node["normalStrength"].get<float>();
  }
  out.albedoTexturePath = ParseString(node, "albedoTexturePath", out.albedoTexturePath);
  out.normalTexturePath = ParseString(node, "normalTexturePath", out.normalTexturePath);
  if (node.contains("culling") && node["culling"].is_object()) {
    const auto& culling = node["culling"];
    if (culling.contains("enabled") && culling["enabled"].is_boolean()) {
      out.culling.enabled = culling["enabled"].get<bool>();
    }
    if (culling.contains("boundsRadius") && culling["boundsRadius"].is_number()) {
      out.culling.boundsRadius = culling["boundsRadius"].get<float>();
    }
  }
  return out;
}

}  // namespace

bool ValidateSceneDescriptorJson(const std::string& path, std::string* error) {
  std::ifstream in(path);
  if (!in.is_open()) {
    if (error != nullptr) {
      *error = "failed to open scene descriptor: " + path;
    }
    return false;
  }

  nlohmann::json root;
  try {
    in >> root;
  } catch (...) {
    if (error != nullptr) {
      *error = "failed to parse scene descriptor JSON";
    }
    return false;
  }
  if (!root.is_object()) {
    if (error != nullptr) {
      *error = "scene descriptor root must be an object";
    }
    return false;
  }

  const bool ok =
      ValidateTopLevelType(root, "drawFieldImage", nlohmann::json::value_t::boolean) &&
      ValidateTopLevelType(root, "drawGrid", nlohmann::json::value_t::boolean) &&
      ValidateTopLevelType(root, "drawAxes", nlohmann::json::value_t::boolean) &&
      ValidateItemObjectArray(root, "overlay") &&
      ValidateItemObjectArray(root, "spheres") &&
      ValidateItemObjectArray(root, "boxes") &&
      ValidateItemObjectArray(root, "lines") &&
      ValidateItemObjectArray(root, "meshes") &&
      ValidateItemObjectArray(root, "entities") &&
      ValidateItemObjectArray(root, "dynamicEntities");
  if (!ok && error != nullptr) {
    *error = "scene descriptor contains invalid top-level key types";
  }
  return ok;
}

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
  std::string validationError;
  if (!ValidateSceneDescriptorJson(path, &validationError)) {
    return std::nullopt;
  }

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

  if (root.contains("spheres") && root["spheres"].is_array()) {
    for (const auto& item : root["spheres"]) {
      if (!item.is_object()) {
        continue;
      }

      SpherePrimitive sphere;
      if (item.contains("center")) {
        sphere.center = ParseVec3(item["center"], sphere.center);
      }
      if (item.contains("radius") && item["radius"].is_number()) {
        sphere.radius = item["radius"].get<float>();
      }
      if (item.contains("color")) {
        sphere.color = ParseColor(item["color"], sphere.color);
      }
      if (item.contains("pass")) {
        sphere.pass = ParseRenderPass(item["pass"], sphere.pass);
      }
      descriptor.staticSpheres.push_back(std::move(sphere));
    }
  }

  if (root.contains("boxes") && root["boxes"].is_array()) {
    for (const auto& item : root["boxes"]) {
      if (!item.is_object()) {
        continue;
      }

      BoxPrimitive box;
      if (item.contains("center")) {
        box.center = ParseVec3(item["center"], box.center);
      }
      if (item.contains("size")) {
        box.size = ParseVec3(item["size"], box.size);
      }
      if (item.contains("yawDeg") && item["yawDeg"].is_number()) {
        box.yawDeg = item["yawDeg"].get<float>();
      }
      if (item.contains("color")) {
        box.color = ParseColor(item["color"], box.color);
      }
      if (item.contains("pass")) {
        box.pass = ParseRenderPass(item["pass"], box.pass);
      }
      descriptor.staticBoxes.push_back(std::move(box));
    }
  }

  if (root.contains("lines") && root["lines"].is_array()) {
    for (const auto& item : root["lines"]) {
      if (!item.is_object()) {
        continue;
      }

      LinePrimitive line;
      if (item.contains("a")) {
        line.a = ParseVec3(item["a"], line.a);
      }
      if (item.contains("b")) {
        line.b = ParseVec3(item["b"], line.b);
      }
      if (item.contains("width") && item["width"].is_number()) {
        line.width = item["width"].get<float>();
      }
      if (item.contains("color")) {
        line.color = ParseColor(item["color"], line.color);
      }
      if (item.contains("pass")) {
        line.pass = ParseRenderPass(item["pass"], line.pass);
      }
      descriptor.staticLines.push_back(std::move(line));
    }
  }

  if (root.contains("meshes") && root["meshes"].is_array()) {
    for (const auto& item : root["meshes"]) {
      if (!item.is_object()) {
        continue;
      }
      if (!item.contains("assetPath") || !item["assetPath"].is_string()) {
        continue;
      }

      MeshInstancePrimitive mesh;
      mesh.assetPath = item["assetPath"].get<std::string>();
      if (item.contains("position")) {
        mesh.position = ParseVec3(item["position"], mesh.position);
      }
      if (item.contains("rotationDeg")) {
        mesh.rotationDeg = ParseVec3(item["rotationDeg"], mesh.rotationDeg);
      }
      if (item.contains("scale")) {
        mesh.scale = ParseVec3(item["scale"], mesh.scale);
      }
      if (item.contains("color")) {
        mesh.color = ParseColor(item["color"], mesh.color);
      }
      if (item.contains("wireframe") && item["wireframe"].is_boolean()) {
        mesh.wireframe = item["wireframe"].get<bool>();
      }
      if (item.contains("roughnessOverride") && item["roughnessOverride"].is_number()) {
        mesh.roughnessOverride = item["roughnessOverride"].get<float>();
      }
      if (item.contains("metallicOverride") && item["metallicOverride"].is_number()) {
        mesh.metallicOverride = item["metallicOverride"].get<float>();
      }
      if (item.contains("normalStrength") && item["normalStrength"].is_number()) {
        mesh.normalStrength = item["normalStrength"].get<float>();
      }
      if (item.contains("albedoTexturePath") && item["albedoTexturePath"].is_string()) {
        mesh.albedoTexturePath = item["albedoTexturePath"].get<std::string>();
      }
      if (item.contains("normalTexturePath") && item["normalTexturePath"].is_string()) {
        mesh.normalTexturePath = item["normalTexturePath"].get<std::string>();
      }
      if (item.contains("pass")) {
        mesh.pass = ParseRenderPass(item["pass"], mesh.pass);
      }
      descriptor.staticMeshes.push_back(std::move(mesh));
    }
  }

  if (root.contains("entities") && root["entities"].is_array()) {
    for (const auto& item : root["entities"]) {
      if (!item.is_object()) {
        continue;
      }
      if (!item.contains("type") || !item["type"].is_string()) {
        continue;
      }

      std::string type = item["type"].get<std::string>();
      std::transform(type.begin(), type.end(), type.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

      RenderEntity entity;
      if (item.contains("id") && item["id"].is_string()) {
        entity.id = item["id"].get<std::string>();
      }
      if (item.contains("pass")) {
        entity.pass = ParseRenderPass(item["pass"], entity.pass);
      }
      if (item.contains("parentId") && item["parentId"].is_string()) {
        entity.parentId = item["parentId"].get<std::string>();
      }
      if (item.contains("transform")) {
        entity.transform = ParseTransformNode(item["transform"], entity.transform);
        entity.hasTransform = true;
      }
      if (item.contains("culling") && item["culling"].is_object()) {
        const auto& culling = item["culling"];
        if (culling.contains("enabled") && culling["enabled"].is_boolean()) {
          entity.culling.enabled = culling["enabled"].get<bool>();
        }
        if (culling.contains("boundsRadius") && culling["boundsRadius"].is_number()) {
          entity.culling.boundsRadius = culling["boundsRadius"].get<float>();
        }
      }

      if (type == "sphere") {
        SpherePrimitive sphere;
        if (item.contains("center")) {
          sphere.center = ParseVec3(item["center"], sphere.center);
        }
        if (item.contains("radius") && item["radius"].is_number()) {
          sphere.radius = item["radius"].get<float>();
        }
        if (item.contains("color")) {
          sphere.color = ParseColor(item["color"], sphere.color);
        }
        sphere.pass = entity.pass;
        entity.payload = std::move(sphere);
      } else if (type == "box") {
        BoxPrimitive box;
        if (item.contains("center")) {
          box.center = ParseVec3(item["center"], box.center);
        }
        if (item.contains("size")) {
          box.size = ParseVec3(item["size"], box.size);
        }
        if (item.contains("yawDeg") && item["yawDeg"].is_number()) {
          box.yawDeg = item["yawDeg"].get<float>();
        }
        if (item.contains("color")) {
          box.color = ParseColor(item["color"], box.color);
        }
        box.pass = entity.pass;
        entity.payload = std::move(box);
      } else if (type == "line") {
        LinePrimitive line;
        if (item.contains("a")) {
          line.a = ParseVec3(item["a"], line.a);
        }
        if (item.contains("b")) {
          line.b = ParseVec3(item["b"], line.b);
        }
        if (item.contains("width") && item["width"].is_number()) {
          line.width = item["width"].get<float>();
        }
        if (item.contains("color")) {
          line.color = ParseColor(item["color"], line.color);
        }
        line.pass = entity.pass;
        entity.payload = std::move(line);
      } else if (type == "mesh") {
        if (!item.contains("assetPath") || !item["assetPath"].is_string()) {
          continue;
        }
        MeshInstancePrimitive mesh;
        mesh.assetPath = item["assetPath"].get<std::string>();
        if (item.contains("position")) {
          mesh.position = ParseVec3(item["position"], mesh.position);
        }
        if (item.contains("rotationDeg")) {
          mesh.rotationDeg = ParseVec3(item["rotationDeg"], mesh.rotationDeg);
        }
        if (item.contains("scale")) {
          mesh.scale = ParseVec3(item["scale"], mesh.scale);
        }
        if (item.contains("color")) {
          mesh.color = ParseColor(item["color"], mesh.color);
        }
        if (item.contains("wireframe") && item["wireframe"].is_boolean()) {
          mesh.wireframe = item["wireframe"].get<bool>();
        }
        if (item.contains("roughnessOverride") && item["roughnessOverride"].is_number()) {
          mesh.roughnessOverride = item["roughnessOverride"].get<float>();
        }
        if (item.contains("metallicOverride") && item["metallicOverride"].is_number()) {
          mesh.metallicOverride = item["metallicOverride"].get<float>();
        }
        if (item.contains("normalStrength") && item["normalStrength"].is_number()) {
          mesh.normalStrength = item["normalStrength"].get<float>();
        }
        if (item.contains("albedoTexturePath") && item["albedoTexturePath"].is_string()) {
          mesh.albedoTexturePath = item["albedoTexturePath"].get<std::string>();
        }
        if (item.contains("normalTexturePath") && item["normalTexturePath"].is_string()) {
          mesh.normalTexturePath = item["normalTexturePath"].get<std::string>();
        }
        mesh.pass = entity.pass;
        entity.payload = std::move(mesh);
      } else if (type == "overlay") {
        if (!item.contains("text") || !item["text"].is_string()) {
          continue;
        }
        OverlayLine overlay;
        overlay.text = item["text"].get<std::string>();
        if (item.contains("color")) {
          overlay.color = ParseColor(item["color"], overlay.color);
        }
        if (item.contains("anchor")) {
          overlay.anchor = ParseAnchor(item["anchor"], overlay.anchor);
        }
        if (item.contains("marginX") && item["marginX"].is_number()) {
          overlay.marginX = item["marginX"].get<float>();
        }
        if (item.contains("marginY") && item["marginY"].is_number()) {
          overlay.marginY = item["marginY"].get<float>();
        }
        entity.pass = RenderPass::Overlay;
        entity.payload = std::move(overlay);
      } else {
        continue;
      }

      descriptor.staticEntities.push_back(std::move(entity));
    }
  }

  if (root.contains("dynamicEntities") && root["dynamicEntities"].is_array()) {
    for (const auto& item : root["dynamicEntities"]) {
      if (!item.is_object()) {
        continue;
      }
      auto binding = ParseDynamicEntityBinding(item);
      if (binding.channel.empty()) {
        continue;
      }
      descriptor.dynamicEntityBindings.push_back(std::move(binding));
    }
  }

  return descriptor;
}

std::optional<SceneDescriptor> LoadSceneDescriptorForProfile(const ViewerConfig& cfg) {
  const std::string descriptorPath = ResolveSceneDescriptorPathForProfile(cfg);
  if (descriptorPath.empty()) {
    return std::nullopt;
  }
  return LoadSceneDescriptorFromFile(descriptorPath);
}

std::string ResolveSceneDescriptorPathForProfile(const ViewerConfig& cfg) {
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

    return candidate;
  }

  return "";
}

}  // namespace repulsor3d
