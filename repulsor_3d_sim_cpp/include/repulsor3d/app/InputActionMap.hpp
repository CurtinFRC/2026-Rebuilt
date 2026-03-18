#pragma once

#include <unordered_map>

namespace repulsor3d {

enum class InputAction {
  kNone,
  kQuit,
  kResetCamera,
  kToggleCameraDebug,
  kToggleTruthFuel,
  kToggleAgeFilter,
  kToggleFieldImage,
  kToggleDebugPanel,
  kToggleDebugCounters,
  kToggleDebugCpu,
  kToggleDebugGpu,
  kToggleDebugAssets,
};

class InputActionMap {
 public:
  static InputActionMap CreateDefault();
  InputAction ResolveKey(int key) const;

 private:
  std::unordered_map<int, InputAction> keyMap_;
};

}  // namespace repulsor3d
