#include "repulsor3d/App.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <utility>

#include <glm/common.hpp>

#include "repulsor3d/config/RuntimeConfigProfile.hpp"
#include "repulsor3d/render/RenderWorldAdapter.hpp"

namespace repulsor3d {
namespace {

bool ParseEnvBool(const char* name, const bool fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  const std::string text(value);
  if (text == "1" || text == "true" || text == "TRUE" || text == "on" || text == "ON") {
    return true;
  }
  if (text == "0" || text == "false" || text == "FALSE" || text == "off" || text == "OFF") {
    return false;
  }
  return fallback;
}

}  // namespace

ViewerApp::ViewerApp(const ViewerConfig& cfg, std::unique_ptr<ISnapshotSource> source)
    : cfg_(cfg),
      source_(std::move(source)),
      camera_(cfg.cameraDistanceM,
              cfg.cameraYawDeg,
              cfg.cameraPitchDeg,
              glm::vec3{cfg.fieldLengthM * 0.5F, cfg.fieldWidthM * 0.5F, 0.0F}),
      renderer_(cfg),
      inputActions_(InputActionMap::CreateDefault()),
      followController_(glm::vec3{cfg.fieldLengthM * 0.5F, cfg.fieldWidthM * 0.5F, 0.0F}),
      domainAdapter_(cfg),
      fixedTicker_(1.0 / std::max(1, cfg.fps)),
      fieldTarget_{cfg.fieldLengthM * 0.5F, cfg.fieldWidthM * 0.5F, 0.0F} {
  renderer_.showCameraDebug = cfg.showCameraDebug;
  renderer_.showTruthFuel = cfg.showTruthFuel;
  renderer_.showAgeFilteredFuel = cfg.showAgeFilteredFuel;
  renderer_.showFieldImage = cfg.showFieldImage;
  renderer_.showDebugPanel = cfg.showDebugPanel;
}

ViewerApp::~ViewerApp() {
  if (worker_ != nullptr) {
    worker_->Stop();
  }
  if (truth_ != nullptr) {
    truth_->Stop();
  }

  if (window_ != nullptr) {
    glfwDestroyWindow(window_);
    window_ = nullptr;
  }
  glfwTerminate();
}

int ViewerApp::Run() {
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    return 1;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#if defined(__APPLE__)
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

  window_ = glfwCreateWindow(cfg_.windowW, cfg_.windowH, "Repulsor 3D NT4 Viewer (C++)", nullptr, nullptr);
  if (window_ == nullptr) {
    std::cerr << "Failed to create window\n";
    return 1;
  }

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(cfg_.vsync ? 1 : 0);
  const bool vsyncAutoDisableEnabled = ParseEnvBool("VSYNC_AUTO_DISABLE", true);
  bool vsyncAutoDisabled = false;
  double swapWaitAverageMs = 0.0;
  bool swapWaitAverageInitialized = false;
  int highSwapWaitFrames = 0;
  int highCpuGpuMismatchFrames = 0;

  glewExperimental = GL_TRUE;
  if (glewInit() != GLEW_OK) {
    std::cerr << "Failed to initialize GLEW\n";
    return 1;
  }

  glGetError();  // clear glew's spurious GL_INVALID_ENUM on core contexts.

  if (!renderer_.Initialize()) {
    std::cerr << "Renderer initialization failed\n";
    return 1;
  }

  glfwSetWindowUserPointer(window_, this);
  glfwSetMouseButtonCallback(window_, &ViewerApp::MouseButtonCallback);
  glfwSetCursorPosCallback(window_, &ViewerApp::CursorPosCallback);
  glfwSetScrollCallback(window_, &ViewerApp::ScrollCallback);
  glfwSetKeyCallback(window_, &ViewerApp::KeyCallback);

  if (cfg_.truthSocketEnabled) {
    truth_ = std::make_unique<TruthSocketReceiver>(cfg_.truthSocketHost, cfg_.truthSocketPort);
    truth_->Start();
  }

  worker_ = std::make_unique<SnapshotWorker>(
      *source_, static_cast<double>(cfg_.fps) * 2.0, cfg_, truth_ != nullptr ? truth_.get() : nullptr);
  worker_->Start();

  auto last = timeSource_.Now();
  auto lastTitleUpdate = timeSource_.Now();

  while (!glfwWindowShouldClose(window_)) {
    glfwPollEvents();

    const auto now = timeSource_.Now();
    const double frameDt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last).count();
    last = now;

    MaybeReloadRuntimeConfigProfile(now);

    fixedTicker_.Advance(frameDt, [this](const double dt) { Tick(dt); });

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window_, &width, &height);
    renderer_.Resize(width, height);

    Draw();

    if (now - lastTitleUpdate > std::chrono::milliseconds(300)) {
      const auto& ex = latest_.snapshot.extrinsics;
      const std::string title = "Ex=" + std::to_string(ex[0]) + "," + std::to_string(ex[1]) + "," +
                                std::to_string(ex[2]) + "," + std::to_string(ex[3]) + "," +
                                std::to_string(ex[4]) + "," + std::to_string(ex[5]);
      glfwSetWindowTitle(window_, title.c_str());
      lastTitleUpdate = now;
    }

    const auto swapStart = std::chrono::steady_clock::now();
    glfwSwapBuffers(window_);
    const auto swapEnd = std::chrono::steady_clock::now();
    const double swapWaitMs =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(swapEnd - swapStart).count();
    if (!swapWaitAverageInitialized) {
      swapWaitAverageMs = swapWaitMs;
      swapWaitAverageInitialized = true;
    } else {
      constexpr double alpha = 0.10;
      swapWaitAverageMs = (1.0 - alpha) * swapWaitAverageMs + alpha * swapWaitMs;
    }
    renderer_.QueueDiagnosticCounter("app.swap_wait_ms", swapWaitMs);
    renderer_.QueueDiagnosticCounter("app.swap_wait_avg_ms", swapWaitAverageMs);
    renderer_.QueueDiagnosticCounter("app.vsync_active", cfg_.vsync ? 1.0 : 0.0);

    if (cfg_.vsync && vsyncAutoDisableEnabled && !vsyncAutoDisabled) {
      const auto& diag = renderer_.LatestDiagnostics();
      const double frameAvgMs = diag.frameAverageMilliseconds > 0.0 ? diag.frameAverageMilliseconds : diag.frameMilliseconds;
      double cadGpuMs = 0.0;
      for (const auto& gpuSample : diag.gpuTimings) {
        if (gpuSample.name == "cad_opaque") {
          cadGpuMs = gpuSample.milliseconds;
          break;
        }
      }
      const bool highSwapWait = swapWaitAverageMs > 8.0;
      const bool lowFps = frameAvgMs > 25.0;
      if (highSwapWait && lowFps) {
        ++highSwapWaitFrames;
      } else {
        highSwapWaitFrames = 0;
      }
      const bool largeCpuGpuGap = cadGpuMs > 0.0 && frameAvgMs > (cadGpuMs * 3.0);
      if (lowFps && largeCpuGpuGap) {
        ++highCpuGpuMismatchFrames;
      } else {
        highCpuGpuMismatchFrames = 0;
      }
      if (highSwapWaitFrames >= 45 || highCpuGpuMismatchFrames >= 45) {
        glfwSwapInterval(0);
        cfg_.vsync = false;
        vsyncAutoDisabled = true;
        renderer_.QueueDiagnosticMessage(
            "auto-disabled vsync (swap_wait_avg_ms=" + std::to_string(swapWaitAverageMs) +
            ", frame_avg_ms=" + std::to_string(frameAvgMs) +
            ", cad_gpu_ms=" + std::to_string(cadGpuMs) + ")");
      }
    }
  }

  return 0;
}

bool ViewerApp::QueryFileWriteTime(const std::string& path, std::filesystem::file_time_type& outWriteTime) {
  if (path.empty()) {
    return false;
  }
  std::error_code ec;
  if (!std::filesystem::exists(path, ec) || ec) {
    return false;
  }
  outWriteTime = std::filesystem::last_write_time(path, ec);
  return !ec;
}

void ViewerApp::ApplyRuntimeConfigProfile(const ViewerConfig& updatedCfg) {
  const int oldFps = std::max(1, cfg_.fps);
  const bool worldAdapterRelevantChange =
      cfg_.sceneProfile != updatedCfg.sceneProfile ||
      cfg_.sceneDescriptorPath != updatedCfg.sceneDescriptorPath ||
      cfg_.seasonModulePluginPath != updatedCfg.seasonModulePluginPath ||
      cfg_.hotReloadSceneDescriptor != updatedCfg.hotReloadSceneDescriptor ||
      cfg_.hotReloadSeasonModule != updatedCfg.hotReloadSeasonModule;

  cfg_ = updatedCfg;
  renderer_.ApplyRuntimeConfig(cfg_);
  domainAdapter_.ApplyConfig(cfg_);
  const int newFps = std::max(1, cfg_.fps);
  if (newFps != oldFps) {
    fixedTicker_.SetFixedStepSeconds(1.0 / static_cast<double>(newFps));
  }
  if (worldAdapterRelevantChange) {
    renderer_.SetRenderWorldAdapter(CreateDefaultRenderWorldAdapter(cfg_));
  }

  fieldTarget_ = glm::vec3{cfg_.fieldLengthM * 0.5F, cfg_.fieldWidthM * 0.5F, 0.0F};
  followController_.Reset(fieldTarget_);
}

void ViewerApp::MaybeReloadRuntimeConfigProfile(const std::chrono::steady_clock::time_point& now) {
  if (!cfg_.hotReloadRuntimeConfigProfile || cfg_.runtimeConfigProfilePath.empty()) {
    return;
  }
  if (nextRuntimeConfigCheck_.time_since_epoch().count() != 0 && now < nextRuntimeConfigCheck_) {
    return;
  }
  nextRuntimeConfigCheck_ = now + std::chrono::milliseconds(500);

  std::filesystem::file_time_type writeTime{};
  const bool stampKnown = QueryFileWriteTime(cfg_.runtimeConfigProfilePath, writeTime);
  if (!stampKnown) {
    return;
  }
  if (runtimeConfigWriteTimeKnown_ && writeTime == runtimeConfigWriteTime_) {
    return;
  }

  ViewerConfig next = cfg_;
  std::string error;
  if (!LoadViewerConfigProfile(cfg_.runtimeConfigProfilePath, next, &error)) {
    std::cerr << "[RuntimeConfig] reload failed for '" << cfg_.runtimeConfigProfilePath << "': " << error << "\n";
    renderer_.QueueDiagnosticMessage("runtime config reload failed: " + error);
    runtimeConfigWriteTime_ = writeTime;
    runtimeConfigWriteTimeKnown_ = true;
    return;
  }

  runtimeConfigWriteTime_ = writeTime;
  runtimeConfigWriteTimeKnown_ = true;
  ApplyRuntimeConfigProfile(next);
  std::cerr << "[RuntimeConfig] reloaded profile: " << cfg_.runtimeConfigProfilePath << "\n";
  renderer_.QueueDiagnosticMessage("runtime config reloaded: " + cfg_.runtimeConfigProfilePath);
}

void ViewerApp::Tick(const double dt) {
  latest_ = worker_->Latest();
  renderSnapshot_ = domainAdapter_.BuildRenderSnapshot(latest_, dt);

  if (cfg_.followRobot && renderSnapshot_.has_value() && renderSnapshot_->pose.has_value()) {
    const glm::vec3 desired = domainAdapter_.ComputeDesiredFollowTarget(*renderSnapshot_, fieldTarget_);
    followController_.Update(camera_, desired, dt, cfg_.followSmoothTimeS, cfg_.followMaxSpeedMps);
  }
}

void ViewerApp::Draw() {
  SnapshotBundle bundle = latest_;
  if (renderSnapshot_.has_value()) {
    bundle.snapshot = *renderSnapshot_;
  }
  SnapshotBundleSimWorld world(std::move(bundle));
  renderer_.Draw(window_, camera_, world);
}

void ViewerApp::OnMouseButton(const int button, const int action) {
  orbitController_.OnMouseButton(button, action);
}

void ViewerApp::OnMouseMove(const double x, const double y) {
  orbitController_.OnMouseMove(x, y, camera_);
}

void ViewerApp::OnScroll(const double yOffset) {
  orbitController_.OnScroll(yOffset, camera_);
}

void ViewerApp::OnKey(const int key, const int action) {
  if (action != GLFW_PRESS) {
    return;
  }

  switch (inputActions_.ResolveKey(key)) {
    case InputAction::kQuit:
      glfwSetWindowShouldClose(window_, GLFW_TRUE);
      break;
    case InputAction::kResetCamera:
      camera_.target = fieldTarget_;
      followController_.Reset(fieldTarget_);
      domainAdapter_.Reset();
      orbitController_.Reset();

      camera_.distance = cfg_.cameraDistanceM;
      camera_.pitchDeg = cfg_.cameraPitchDeg;
      camera_.yawDeg = cfg_.cameraYawDeg;
      break;
    case InputAction::kToggleCameraDebug:
      renderer_.showCameraDebug = !renderer_.showCameraDebug;
      break;
    case InputAction::kToggleTruthFuel:
      renderer_.showTruthFuel = !renderer_.showTruthFuel;
      break;
    case InputAction::kToggleAgeFilter:
      renderer_.showAgeFilteredFuel = !renderer_.showAgeFilteredFuel;
      break;
    case InputAction::kToggleFieldImage:
      renderer_.showFieldImage = !renderer_.showFieldImage;
      break;
    case InputAction::kToggleDebugPanel:
      renderer_.showDebugPanel = !renderer_.showDebugPanel;
      break;
    case InputAction::kToggleDebugCounters:
      renderer_.showDebugCounters = !renderer_.showDebugCounters;
      break;
    case InputAction::kToggleDebugCpu:
      renderer_.showDebugCpu = !renderer_.showDebugCpu;
      break;
    case InputAction::kToggleDebugGpu:
      renderer_.showDebugGpu = !renderer_.showDebugGpu;
      break;
    case InputAction::kToggleDebugAssets:
      renderer_.showDebugAssets = !renderer_.showDebugAssets;
      break;
    default:
      break;
  }
}

void ViewerApp::MouseButtonCallback(GLFWwindow* window, const int button, const int action, const int /*mods*/) {
  auto* app = static_cast<ViewerApp*>(glfwGetWindowUserPointer(window));
  if (app != nullptr) {
    app->OnMouseButton(button, action);
  }
}

void ViewerApp::CursorPosCallback(GLFWwindow* window, const double x, const double y) {
  auto* app = static_cast<ViewerApp*>(glfwGetWindowUserPointer(window));
  if (app != nullptr) {
    app->OnMouseMove(x, y);
  }
}

void ViewerApp::ScrollCallback(GLFWwindow* window, const double /*xOffset*/, const double yOffset) {
  auto* app = static_cast<ViewerApp*>(glfwGetWindowUserPointer(window));
  if (app != nullptr) {
    app->OnScroll(yOffset);
  }
}

void ViewerApp::KeyCallback(GLFWwindow* window, const int key, const int /*scancode*/, const int action, const int /*mods*/) {
  auto* app = static_cast<ViewerApp*>(glfwGetWindowUserPointer(window));
  if (app != nullptr) {
    app->OnKey(key, action);
  }
}

}  // namespace repulsor3d
