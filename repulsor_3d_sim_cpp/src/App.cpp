#include "repulsor3d/App.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <utility>

#include <glm/common.hpp>

#include "repulsor3d/MathUtil.hpp"

namespace repulsor3d {

ViewerApp::ViewerApp(const ViewerConfig& cfg, std::unique_ptr<ISnapshotSource> source)
    : cfg_(cfg),
      source_(std::move(source)),
      camera_(cfg.cameraDistanceM,
              cfg.cameraYawDeg,
              cfg.cameraPitchDeg,
              glm::vec3{cfg.fieldLengthM * 0.5F, cfg.fieldWidthM * 0.5F, 0.0F}),
      renderer_(cfg),
      fieldTarget_{cfg.fieldLengthM * 0.5F, cfg.fieldWidthM * 0.5F, 0.0F},
      followTarget_(fieldTarget_) {
  renderer_.showCameraDebug = cfg.showCameraDebug;
  renderer_.showTruthFuel = cfg.showTruthFuel;
  renderer_.showAgeFilteredFuel = cfg.showAgeFilteredFuel;
  renderer_.showFieldImage = cfg.showFieldImage;
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
  glfwSwapInterval(1);

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
      *source_, static_cast<double>(cfg_.fps) * 2.0, truth_ != nullptr ? truth_.get() : nullptr);
  worker_->Start();

  auto last = std::chrono::steady_clock::now();
  double accumulator = 0.0;
  const double fixedDt = 1.0 / std::max(1, cfg_.fps);

  auto lastTitleUpdate = std::chrono::steady_clock::now();

  while (!glfwWindowShouldClose(window_)) {
    glfwPollEvents();

    const auto now = std::chrono::steady_clock::now();
    const double frameDt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last).count();
    last = now;

    accumulator = std::min(0.5, accumulator + frameDt);
    while (accumulator >= fixedDt) {
      Tick(fixedDt);
      accumulator -= fixedDt;
    }

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

    glfwSwapBuffers(window_);
  }

  return 0;
}

void ViewerApp::Tick(const double dt) {
  latest_ = worker_->Latest();
  renderSnapshot_ = latest_.snapshot;

  if (renderSnapshot_.has_value() && renderSnapshot_->pose.has_value()) {
    const double px = renderSnapshot_->pose->x;
    const double py = renderSnapshot_->pose->y;
    const double ph = renderSnapshot_->pose->thetaRad;

    if (!robotX_.has_value() || !robotY_.has_value() || !robotH_.has_value()) {
      robotX_ = px;
      robotY_ = py;
      robotH_ = ph;
      robotVx_ = 0.0;
      robotVy_ = 0.0;
      robotVh_ = 0.0;
    } else {
      std::tie(*robotX_, robotVx_) =
          SmoothDamp(*robotX_, px, robotVx_, static_cast<double>(cfg_.robotSmoothTimeS), dt,
                     static_cast<double>(cfg_.robotMaxSpeedMps));
      std::tie(*robotY_, robotVy_) =
          SmoothDamp(*robotY_, py, robotVy_, static_cast<double>(cfg_.robotSmoothTimeS), dt,
                     static_cast<double>(cfg_.robotMaxSpeedMps));
      std::tie(*robotH_, robotVh_) = SmoothDampAngle(*robotH_, ph, robotVh_, static_cast<double>(cfg_.robotSmoothTimeS),
                                                     dt, 50.0);
    }

    Pose2D smoothed = renderSnapshot_->pose.value();
    smoothed.x = *robotX_;
    smoothed.y = *robotY_;
    smoothed.thetaRad = *robotH_;
    renderSnapshot_->pose = smoothed;
  }

  if (cfg_.followRobot && renderSnapshot_.has_value() && renderSnapshot_->pose.has_value()) {
    const glm::vec3 desired{static_cast<float>(renderSnapshot_->pose->x),
                            static_cast<float>(renderSnapshot_->pose->y), 0.0F};

    const auto [nx, vx] =
        SmoothDamp(followTarget_.x, desired.x, followVel_.x, cfg_.followSmoothTimeS, dt, cfg_.followMaxSpeedMps);
    const auto [ny, vy] =
        SmoothDamp(followTarget_.y, desired.y, followVel_.y, cfg_.followSmoothTimeS, dt, cfg_.followMaxSpeedMps);
    const auto [nz, vz] =
        SmoothDamp(followTarget_.z, desired.z, followVel_.z, cfg_.followSmoothTimeS, dt, cfg_.followMaxSpeedMps);

    followTarget_.x = static_cast<float>(nx);
    followTarget_.y = static_cast<float>(ny);
    followTarget_.z = static_cast<float>(nz);
    followVel_.x = static_cast<float>(vx);
    followVel_.y = static_cast<float>(vy);
    followVel_.z = static_cast<float>(vz);

    camera_.target = followTarget_;
  }
}

void ViewerApp::Draw() {
  SnapshotBundle bundle = latest_;
  if (renderSnapshot_.has_value()) {
    bundle.snapshot = *renderSnapshot_;
  }
  renderer_.Draw(window_, camera_, bundle);
}

void ViewerApp::OnMouseButton(const int button, const int action) {
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    dragging_ = (action == GLFW_PRESS);
  }
}

void ViewerApp::OnMouseMove(const double x, const double y) {
  if (dragging_) {
    const double dx = x - lastMouseX_;
    const double dy = y - lastMouseY_;

    camera_.yawDeg = std::fmod(camera_.yawDeg + static_cast<float>(dx) * 0.35F, 360.0F);
    camera_.pitchDeg = glm::clamp(camera_.pitchDeg + static_cast<float>(dy) * 0.35F, -89.0F, 89.0F);
  }

  lastMouseX_ = x;
  lastMouseY_ = y;
}

void ViewerApp::OnScroll(const double yOffset) {
  const float s = 1.0F - static_cast<float>(yOffset) * 0.08F;
  const float d = camera_.distance * s;
  camera_.distance = glm::clamp(d, 1.5F, 80.0F);
}

void ViewerApp::OnKey(const int key, const int action) {
  if (action != GLFW_PRESS) {
    return;
  }

  switch (key) {
    case GLFW_KEY_ESCAPE:
      glfwSetWindowShouldClose(window_, GLFW_TRUE);
      break;
    case GLFW_KEY_R:
      camera_.target = fieldTarget_;
      followTarget_ = fieldTarget_;
      followVel_ = glm::vec3{0.0F, 0.0F, 0.0F};

      robotX_.reset();
      robotY_.reset();
      robotH_.reset();
      robotVx_ = 0.0;
      robotVy_ = 0.0;
      robotVh_ = 0.0;

      camera_.distance = cfg_.cameraDistanceM;
      camera_.pitchDeg = cfg_.cameraPitchDeg;
      camera_.yawDeg = cfg_.cameraYawDeg;
      break;
    case GLFW_KEY_C:
      renderer_.showCameraDebug = !renderer_.showCameraDebug;
      break;
    case GLFW_KEY_T:
      renderer_.showTruthFuel = !renderer_.showTruthFuel;
      break;
    case GLFW_KEY_A:
      renderer_.showAgeFilteredFuel = !renderer_.showAgeFilteredFuel;
      break;
    case GLFW_KEY_F:
      renderer_.showFieldImage = !renderer_.showFieldImage;
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
