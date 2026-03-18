#include "repulsor3d/app/InputActionMap.hpp"

#include <GLFW/glfw3.h>

namespace repulsor3d {

InputActionMap InputActionMap::CreateDefault() {
  InputActionMap map;
  map.keyMap_[GLFW_KEY_ESCAPE] = InputAction::kQuit;
  map.keyMap_[GLFW_KEY_R] = InputAction::kResetCamera;
  map.keyMap_[GLFW_KEY_C] = InputAction::kToggleCameraDebug;
  map.keyMap_[GLFW_KEY_T] = InputAction::kToggleTruthFuel;
  map.keyMap_[GLFW_KEY_A] = InputAction::kToggleAgeFilter;
  map.keyMap_[GLFW_KEY_F] = InputAction::kToggleFieldImage;
  map.keyMap_[GLFW_KEY_D] = InputAction::kToggleDebugPanel;
  map.keyMap_[GLFW_KEY_1] = InputAction::kToggleDebugCounters;
  map.keyMap_[GLFW_KEY_2] = InputAction::kToggleDebugCpu;
  map.keyMap_[GLFW_KEY_3] = InputAction::kToggleDebugGpu;
  map.keyMap_[GLFW_KEY_4] = InputAction::kToggleDebugAssets;
  return map;
}

InputAction InputActionMap::ResolveKey(const int key) const {
  const auto it = keyMap_.find(key);
  if (it == keyMap_.end()) {
    return InputAction::kNone;
  }
  return it->second;
}

}  // namespace repulsor3d
