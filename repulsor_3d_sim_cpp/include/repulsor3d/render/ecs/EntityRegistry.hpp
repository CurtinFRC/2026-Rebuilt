#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "repulsor3d/render/SceneFrame.hpp"

namespace repulsor3d {

using RenderEntityId = std::uint64_t;

class RenderEntityRegistry {
 public:
  RenderEntityId CreateEntity(const std::string& debugId = "");
  void SetPass(RenderEntityId id, RenderPass pass);

  void SetSphere(RenderEntityId id, const SpherePrimitive& sphere);
  void SetBox(RenderEntityId id, const BoxPrimitive& box);
  void SetLine(RenderEntityId id, const LinePrimitive& line);
  void SetMesh(RenderEntityId id, const MeshInstancePrimitive& mesh);
  void SetOverlay(RenderEntityId id, const OverlayLine& overlay);

  void Clear();
  std::vector<RenderEntity> BuildEntities() const;

 private:
  int IndexOf(RenderEntityId id) const;

  RenderEntityId nextId_ = 1;
  std::vector<RenderEntityId> ids_;
  std::vector<RenderEntity> entities_;
};

}  // namespace repulsor3d

