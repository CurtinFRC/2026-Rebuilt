#pragma once

#include <glm/vec3.hpp>

#include "repulsor3d/Camera.hpp"

namespace repulsor3d {

class ICameraController {
 public:
  virtual ~ICameraController() = default;
};

class OrbitMouseCameraController final : public ICameraController {
 public:
  void OnMouseButton(int button, int action);
  void OnMouseMove(double x, double y, OrbitCamera& camera);
  void OnScroll(double yOffset, OrbitCamera& camera) const;
  void Reset();

 private:
  bool dragging_ = false;
  double lastMouseX_ = 0.0;
  double lastMouseY_ = 0.0;
};

class FollowTargetCameraController final : public ICameraController {
 public:
  explicit FollowTargetCameraController(glm::vec3 initialTarget);

  void Reset(glm::vec3 target);
  void Update(OrbitCamera& camera, const glm::vec3& desiredTarget, double dt, float smoothTimeS, float maxSpeedMps);
  const glm::vec3& CurrentTarget() const;

 private:
  glm::vec3 target_;
  glm::vec3 velocity_{0.0F, 0.0F, 0.0F};
};

}  // namespace repulsor3d
