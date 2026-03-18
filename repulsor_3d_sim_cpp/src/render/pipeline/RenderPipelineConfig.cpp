#include "repulsor3d/render/pipeline/RenderPipelineConfig.hpp"

#include <fstream>
#include <functional>
#include <unordered_map>
#include <unordered_set>

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

RenderPipelineValidationResult ValidateRenderPipelineConfig(const RenderPipelineConfig& config) {
  RenderPipelineValidationResult result;
  if (config.passes.empty()) {
    result.errors.push_back("pipeline contains no passes");
    return result;
  }

  std::unordered_map<std::string, std::size_t> nameToIndex;
  nameToIndex.reserve(config.passes.size());
  for (std::size_t i = 0; i < config.passes.size(); ++i) {
    const auto& pass = config.passes[i];
    if (pass.name.empty()) {
      result.errors.push_back("pass[" + std::to_string(i) + "] has empty name");
      continue;
    }
    if (pass.factory.empty()) {
      result.errors.push_back("pass[" + pass.name + "] has empty factory");
    }
    if (!nameToIndex.emplace(pass.name, i).second) {
      result.errors.push_back("duplicate pass name '" + pass.name + "'");
    }
  }

  for (const auto& pass : config.passes) {
    for (const auto& dep : pass.dependencies) {
      if (nameToIndex.find(dep) == nameToIndex.end()) {
        result.errors.push_back("pass '" + pass.name + "' depends on unknown pass '" + dep + "'");
      }
    }
  }

  std::unordered_set<std::string> visiting;
  std::unordered_set<std::string> visited;
  std::function<void(const std::string&)> dfs = [&](const std::string& name) {
    if (visited.contains(name) || !nameToIndex.contains(name)) {
      return;
    }
    if (visiting.contains(name)) {
      result.errors.push_back("cyclic dependency at pass '" + name + "'");
      return;
    }

    visiting.insert(name);
    const auto& pass = config.passes[nameToIndex.at(name)];
    for (const auto& dep : pass.dependencies) {
      dfs(dep);
    }
    visiting.erase(name);
    visited.insert(name);
  };

  for (const auto& pass : config.passes) {
    dfs(pass.name);
  }

  result.ok = result.errors.empty();
  return result;
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

  const auto validation = ValidateRenderPipelineConfig(parsed);
  if (!validation.ok) {
    if (outError != nullptr) {
      std::string message;
      for (std::size_t i = 0; i < validation.errors.size(); ++i) {
        if (i > 0) {
          message += "; ";
        }
        message += validation.errors[i];
      }
      *outError = std::move(message);
    }
    return false;
  }

  outConfig = std::move(parsed);
  return true;
}

}  // namespace repulsor3d
