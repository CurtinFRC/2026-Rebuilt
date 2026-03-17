#pragma once

#include <glm/vec4.hpp>

namespace repulsor3d {

struct MaterialDefinition {
  glm::vec4 baseColor{1.0F, 1.0F, 1.0F, 1.0F};
  float roughness = 0.7F;
  float metallic = 0.0F;
  bool wireframe = false;
};

struct MaterialInstance {
  MaterialDefinition definition;
};

class IMaterialPass {
 public:
  virtual ~IMaterialPass() = default;
  virtual void Apply(const MaterialInstance& material) = 0;
};

}  // namespace repulsor3d
