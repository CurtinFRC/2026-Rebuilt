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

RenderPipelineConfig MakeDefaultRenderPipelineConfig();
bool LoadRenderPipelineConfigFromFile(const std::string& filePath, RenderPipelineConfig& outConfig, std::string* outError);

}  // namespace repulsor3d
