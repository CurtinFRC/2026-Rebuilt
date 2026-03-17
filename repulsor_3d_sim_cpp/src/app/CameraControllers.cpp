#include "repulsor3d/app/CameraControllers.hpp"

#include <GLFW/glfw3.h>

#include <cmath>

#include <glm/common.hpp>

#include "repulsor3d/MathUtil.hpp"

namespace repulsor3d {

void OrbitMouseCameraController::OnMouseButton(const int button, const int action) {
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    dragging_ = (action == GLFW_PRESS);
  }
}

void OrbitMouseCameraController::OnMouseMove(const double x, const double y, OrbitCamera& camera) {
  if (dragging_) {
    const double dx = x - lastMouseX_;
    const double dy = y - lastMouseY_;

    camera.yawDeg = std::fmod(camera.yawDeg + static_cast<float>(dx) * 0.35F, 360.0F);
    camera.pitchDeg = glm::clamp(camera.pitchDeg + static_cast<float>(dy) * 0.35F, -89.0F, 89.0F);
  }

  lastMouseX_ = x;
  lastMouseY_ = y;
}

void OrbitMouseCameraController::OnScroll(const double yOffset, OrbitCamera& camera) const {
  const float scale = 1.0F - static_cast<float>(yOffset) * 0.08F;
  const float nextDistance = camera.distance * scale;
  camera.distance = glm::clamp(nextDistance, 1.5F, 80.0F);
}

void OrbitMouseCameraController::Reset() {
  dragging_ = false;
}

FollowTargetCameraController::FollowTargetCameraController(const glm::vec3 initialTarget) : target_(initialTarget) {}

void FollowTargetCameraController::Reset(const glm::vec3 target) {
  target_ = target;
  velocity_ = {0.0F, 0.0F, 0.0F};
}

void FollowTargetCameraController::Update(
    OrbitCamera& camera,
    const glm::vec3& desiredTarget,
    const double dt,
    const float smoothTimeS,
    const float maxSpeedMps) {
  const auto [nx, vx] = SmoothDamp(target_.x, desiredTarget.x, velocity_.x, smoothTimeS, dt, maxSpeedMps);
  const auto [ny, vy] = SmoothDamp(target_.y, desiredTarget.y, velocity_.y, smoothTimeS, dt, maxSpeedMps);
  const auto [nz, vz] = SmoothDamp(target_.z, desiredTarget.z, velocity_.z, smoothTimeS, dt, maxSpeedMps);

  target_.x = static_cast<float>(nx);
  target_.y = static_cast<float>(ny);
  target_.z = static_cast<float>(nz);

  velocity_.x = static_cast<float>(vx);
  velocity_.y = static_cast<float>(vy);
  velocity_.z = static_cast<float>(vz);

  camera.target = target_;
}

const glm::vec3& FollowTargetCameraController::CurrentTarget() const {
  return target_;
}

}  // namespace repulsor3d
