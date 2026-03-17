#pragma once

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

namespace repulsor3d {

class OrbitCamera {
 public:
  OrbitCamera(float distance, float yawDeg, float pitchDeg, glm::vec3 target);

  glm::vec3 Eye() const;

  glm::mat4 ViewMatrix() const;
  glm::mat4 ProjectionMatrix(float aspect, float fovDeg = 60.0F, float nearPlane = 0.05F,
                             float farPlane = 500.0F) const;

  float distance = 12.0F;
  float yawDeg = 135.0F;
  float pitchDeg = 35.0F;
  glm::vec3 target{0.0F, 0.0F, 0.0F};
};

}  // namespace repulsor3d
