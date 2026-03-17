#pragma once

#include <optional>
#include <string>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/render/SceneFrame.hpp"

namespace repulsor3d {

struct SceneDescriptor {
  std::optional<bool> drawFieldImage;
  std::optional<bool> drawGrid;
  std::optional<bool> drawAxes;
  std::vector<OverlayLine> staticOverlayLines;
  std::vector<SpherePrimitive> staticSpheres;
  std::vector<BoxPrimitive> staticBoxes;
  std::vector<LinePrimitive> staticLines;
  std::vector<MeshInstancePrimitive> staticMeshes;
  std::vector<RenderEntity> staticEntities;
};

std::string CanonicalSceneProfileKey(const std::string& sceneProfile);
std::optional<SceneDescriptor> LoadSceneDescriptorFromFile(const std::string& path);
std::optional<SceneDescriptor> LoadSceneDescriptorForProfile(const ViewerConfig& cfg);

}  // namespace repulsor3d
