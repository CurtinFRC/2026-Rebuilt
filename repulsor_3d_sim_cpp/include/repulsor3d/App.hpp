#pragma once

#include <memory>
#include <optional>
#include <chrono>
#include <filesystem>

#include <glm/vec3.hpp>

#include "repulsor3d/app/CameraControllers.hpp"
#include "repulsor3d/app/InputActionMap.hpp"
#include "repulsor3d/Camera.hpp"
#include "repulsor3d/Config.hpp"
#include "repulsor3d/DataSource.hpp"
#include "repulsor3d/domain/SnapshotDomainAdapter.hpp"
#include "repulsor3d/Model.hpp"
#include "repulsor3d/Renderer.hpp"
#include "repulsor3d/SnapshotWorker.hpp"
#include "repulsor3d/time/FixedStepTicker.hpp"
#include "repulsor3d/time/TimeSource.hpp"
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
  void MaybeReloadRuntimeConfigProfile(const std::chrono::steady_clock::time_point& now);
  void ApplyRuntimeConfigProfile(const ViewerConfig& updatedCfg);
  static bool QueryFileWriteTime(const std::string& path, std::filesystem::file_time_type& outWriteTime);

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

  InputActionMap inputActions_;
  OrbitMouseCameraController orbitController_;
  FollowTargetCameraController followController_;
  SnapshotDomainAdapter domainAdapter_;
  SteadyTimeSource timeSource_;
  FixedStepTicker fixedTicker_;

  glm::vec3 fieldTarget_;
  std::filesystem::file_time_type runtimeConfigWriteTime_{};
  bool runtimeConfigWriteTimeKnown_ = false;
  std::chrono::steady_clock::time_point nextRuntimeConfigCheck_{};
};

}  // namespace repulsor3d
