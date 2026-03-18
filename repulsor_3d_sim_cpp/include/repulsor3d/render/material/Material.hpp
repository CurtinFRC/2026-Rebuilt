#pragma once

#include <string>
#include <unordered_map>
#include <variant>

#include <glm/vec4.hpp>

namespace repulsor3d {

using MaterialParameterValue = std::variant<float, int, bool, glm::vec4, std::string>;

struct ShaderPermutationKey {
  int shadingModel = 0;
  bool useNormalMap = false;
  bool useVertexColor = false;
  bool instanced = false;

  bool operator==(const ShaderPermutationKey& other) const {
    return shadingModel == other.shadingModel &&
           useNormalMap == other.useNormalMap &&
           useVertexColor == other.useVertexColor &&
           instanced == other.instanced;
  }
};

struct MaterialDefinition {
  glm::vec4 baseColor{1.0F, 1.0F, 1.0F, 1.0F};
  float roughness = 0.7F;
  float metallic = 0.0F;
  bool wireframe = false;
};

struct MaterialTemplate {
  std::string id;
  MaterialDefinition defaults;
  ShaderPermutationKey permutation;
  std::unordered_map<std::string, MaterialParameterValue> defaultParameters;
};

struct MaterialInstance {
  std::string templateId;
  MaterialDefinition definition;
  ShaderPermutationKey permutation;
  std::unordered_map<std::string, MaterialParameterValue> parameters;
};

class IMaterialPass {
 public:
  virtual ~IMaterialPass() = default;
  virtual void Apply(const MaterialInstance& material) = 0;
};

class IMaterialResolver {
 public:
  virtual ~IMaterialResolver() = default;
  virtual MaterialInstance Resolve(const MaterialTemplate& materialTemplate) const = 0;
};

}  // namespace repulsor3d
