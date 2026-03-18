#pragma once

#include <optional>
#include <string>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/render/SceneFrame.hpp"

namespace repulsor3d {

struct SceneDescriptor {
  struct DynamicEntityBinding {
    std::string channel;
    std::string entityType = "sphere";
    std::string idPrefix = "dyn_";
    RenderPass pass = RenderPass::Opaque;

    std::string xKey = "x";
    std::string yKey = "y";
    std::string zKey = "z";
    std::string yawDegKey = "yaw_deg";
    std::string textKey = "label";

    float defaultRadius = 0.15F;
    glm::vec3 defaultSize{0.4F, 0.4F, 0.4F};
    glm::vec3 defaultScale{1.0F, 1.0F, 1.0F};
    glm::vec4 color{0.95F, 0.35F, 0.2F, 0.8F};
    std::string assetPath;
    bool wireframe = false;
    bool useAssetColor = false;
    EntityCulling culling;
  };

  std::optional<bool> drawFieldImage;
  std::optional<bool> drawGrid;
  std::optional<bool> drawAxes;
  std::vector<OverlayLine> staticOverlayLines;
  std::vector<SpherePrimitive> staticSpheres;
  std::vector<BoxPrimitive> staticBoxes;
  std::vector<LinePrimitive> staticLines;
  std::vector<MeshInstancePrimitive> staticMeshes;
  std::vector<RenderEntity> staticEntities;
  std::vector<DynamicEntityBinding> dynamicEntityBindings;
};

std::string CanonicalSceneProfileKey(const std::string& sceneProfile);
std::string ResolveSceneDescriptorPathForProfile(const ViewerConfig& cfg);
std::optional<SceneDescriptor> LoadSceneDescriptorFromFile(const std::string& path);
std::optional<SceneDescriptor> LoadSceneDescriptorForProfile(const ViewerConfig& cfg);

}  // namespace repulsor3d
