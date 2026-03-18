#pragma once

#include <string>
#include <vector>

namespace repulsor3d {

struct RenderPipelinePassSpec {
  std::string factory;
  std::string name;
  std::vector<std::string> dependencies;
  std::string renderPass;
  bool enabled = true;
};

struct RenderPipelineConfig {
  std::vector<RenderPipelinePassSpec> passes;
};

struct RenderPipelineValidationResult {
  bool ok = false;
  std::vector<std::string> errors;
};

RenderPipelineConfig MakeDefaultRenderPipelineConfig();
RenderPipelineValidationResult ValidateRenderPipelineConfig(const RenderPipelineConfig& config);
bool LoadRenderPipelineConfigFromFile(const std::string& filePath, RenderPipelineConfig& outConfig, std::string* outError);

}  // namespace repulsor3d
