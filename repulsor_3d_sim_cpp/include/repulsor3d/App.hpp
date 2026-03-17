#pragma once

#include <memory>
#include <optional>

#include <glm/vec3.hpp>

#include "repulsor3d/Camera.hpp"
#include "repulsor3d/Config.hpp"
#include "repulsor3d/DataSource.hpp"
#include "repulsor3d/Model.hpp"
#include "repulsor3d/Renderer.hpp"
#include "repulsor3d/SnapshotWorker.hpp"
#include "repulsor3d/TruthSocketReceiver.hpp"

struct GLFWwindow;

namespace repulsor3d {

class ViewerApp {
 public:
  ViewerApp(const ViewerConfig& cfg, std::unique_ptr<ISnapshotSource> source);
  ~ViewerApp();

  ViewerApp(const ViewerApp&) = delete;
  ViewerApp& operator=(const ViewerApp&) = delete;

  int Run();

 private:
  void Tick(double dt);
  void Draw();

  void OnMouseButton(int button, int action);
  void OnMouseMove(double x, double y);
  void OnScroll(double yOffset);
  void OnKey(int key, int action);

  static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
  static void CursorPosCallback(GLFWwindow* window, double x, double y);
  static void ScrollCallback(GLFWwindow* window, double xOffset, double yOffset);
  static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

  ViewerConfig cfg_;
  std::unique_ptr<ISnapshotSource> source_;
  std::unique_ptr<TruthSocketReceiver> truth_;
  std::unique_ptr<SnapshotWorker> worker_;

  GLFWwindow* window_ = nullptr;
  OrbitCamera camera_;
  Renderer renderer_;

  SnapshotBundle latest_;
  std::optional<WorldSnapshot> renderSnapshot_;

  bool dragging_ = false;
  double lastMouseX_ = 0.0;
  double lastMouseY_ = 0.0;

  glm::vec3 fieldTarget_;
  glm::vec3 followTarget_;
  glm::vec3 followVel_{0.0F, 0.0F, 0.0F};

  std::optional<double> robotX_;
  std::optional<double> robotY_;
  std::optional<double> robotH_;
  double robotVx_ = 0.0;
  double robotVy_ = 0.0;
  double robotVh_ = 0.0;
};

}  // namespace repulsor3d
