#include "repulsor3d/Camera.hpp"

#include <algorithm>
#include <cmath>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>

namespace repulsor3d {

OrbitCamera::OrbitCamera(const float distanceIn, const float yawDegIn, const float pitchDegIn, const glm::vec3 targetIn)
    : distance(distanceIn), yawDeg(yawDegIn), pitchDeg(pitchDegIn), target(targetIn) {}

glm::vec3 OrbitCamera::Eye() const {
  const float yaw = glm::radians(yawDeg);
  const float pitch = glm::radians(pitchDeg);
  return {
      target.x + distance * std::cos(pitch) * std::cos(yaw),
      target.y + distance * std::cos(pitch) * std::sin(yaw),
      target.z + distance * std::sin(pitch),
  };
}

glm::mat4 OrbitCamera::ViewMatrix() const {
  return glm::lookAt(Eye(), target, glm::vec3(0.0F, 0.0F, 1.0F));
}

glm::mat4 OrbitCamera::ProjectionMatrix(
    const float aspect,
    const float fovDeg,
    const float nearPlane,
    const float farPlane) const {
  return glm::perspective(glm::radians(fovDeg), std::max(1e-3F, aspect), nearPlane, farPlane);
}

}  // namespace repulsor3d
