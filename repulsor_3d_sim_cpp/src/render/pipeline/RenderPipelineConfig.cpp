#include "repulsor3d/render/pipeline/RenderPipelineConfig.hpp"

#include <fstream>

#include <nlohmann/json.hpp>

namespace repulsor3d {

RenderPipelineConfig MakeDefaultRenderPipelineConfig() {
  RenderPipelineConfig cfg;
  cfg.passes = {
      {.factory = "world", .name = "world", .dependencies = {}, .renderPass = "", .enabled = true},
      {.factory = "geometry", .name = "geometry_opaque", .dependencies = {"world"}, .renderPass = "opaque", .enabled = true},
      {.factory = "cad", .name = "cad_opaque", .dependencies = {"geometry_opaque"}, .renderPass = "opaque", .enabled = true},
      {.factory = "geometry", .name = "geometry_transparent", .dependencies = {"cad_opaque"}, .renderPass = "transparent", .enabled = true},
      {.factory = "cad", .name = "cad_transparent", .dependencies = {"geometry_transparent"}, .renderPass = "transparent", .enabled = true},
      {.factory = "overlay", .name = "overlay", .dependencies = {"cad_transparent"}, .renderPass = "", .enabled = true},
  };
  return cfg;
}

bool LoadRenderPipelineConfigFromFile(const std::string& filePath, RenderPipelineConfig& outConfig, std::string* outError) {
  if (filePath.empty()) {
    if (outError != nullptr) {
      *outError = "empty path";
    }
    return false;
  }

  std::ifstream in(filePath);
  if (!in.is_open()) {
    if (outError != nullptr) {
      *outError = "failed to open file";
    }
    return false;
  }

  nlohmann::json root;
  try {
    in >> root;
  } catch (...) {
    if (outError != nullptr) {
      *outError = "invalid json";
    }
    return false;
  }

  if (!root.is_object() || !root.contains("passes") || !root["passes"].is_array()) {
    if (outError != nullptr) {
      *outError = "missing passes[]";
    }
    return false;
  }

  RenderPipelineConfig parsed;
  for (const auto& pass : root["passes"]) {
    if (!pass.is_object()) {
      continue;
    }
    RenderPipelinePassSpec spec;
    spec.factory = pass.value("factory", "");
    spec.name = pass.value("name", "");
    spec.renderPass = pass.value("renderPass", "");
    spec.enabled = pass.value("enabled", true);
    if (pass.contains("dependencies") && pass["dependencies"].is_array()) {
      for (const auto& dep : pass["dependencies"]) {
        if (dep.is_string()) {
          spec.dependencies.push_back(dep.get<std::string>());
        }
      }
    }
    if (!spec.factory.empty() && !spec.name.empty()) {
      parsed.passes.push_back(std::move(spec));
    }
  }

  if (parsed.passes.empty()) {
    if (outError != nullptr) {
      *outError = "no valid passes";
    }
    return false;
  }

  outConfig = std::move(parsed);
  return true;
}

}  // namespace repulsor3d
