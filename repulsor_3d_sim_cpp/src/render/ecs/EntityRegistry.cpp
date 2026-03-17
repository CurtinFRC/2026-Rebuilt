#include "repulsor3d/render/ecs/EntityRegistry.hpp"

namespace repulsor3d {

RenderEntityId RenderEntityRegistry::CreateEntity(const std::string& debugId) {
  const RenderEntityId id = nextId_++;
  ids_.push_back(id);
  RenderEntity entity;
  entity.id = debugId.empty() ? std::to_string(id) : debugId;
  entities_.push_back(std::move(entity));
  return id;
}

void RenderEntityRegistry::SetPass(const RenderEntityId id, const RenderPass pass) {
  const int idx = IndexOf(id);
  if (idx < 0) {
    return;
  }
  entities_[static_cast<size_t>(idx)].pass = pass;
}

void RenderEntityRegistry::SetParent(const RenderEntityId id, const std::string& parentDebugId) {
  const int idx = IndexOf(id);
  if (idx < 0) {
    return;
  }
  entities_[static_cast<size_t>(idx)].parentId = parentDebugId;
}

void RenderEntityRegistry::SetTransform(const RenderEntityId id, const Transform3D& transform) {
  const int idx = IndexOf(id);
  if (idx < 0) {
    return;
  }
  entities_[static_cast<size_t>(idx)].transform = transform;
  entities_[static_cast<size_t>(idx)].hasTransform = true;
}

void RenderEntityRegistry::ClearTransform(const RenderEntityId id) {
  const int idx = IndexOf(id);
  if (idx < 0) {
    return;
  }
  entities_[static_cast<size_t>(idx)].transform = Transform3D{};
  entities_[static_cast<size_t>(idx)].hasTransform = false;
}

void RenderEntityRegistry::SetCulling(const RenderEntityId id, const EntityCulling& culling) {
  const int idx = IndexOf(id);
  if (idx < 0) {
    return;
  }
  entities_[static_cast<size_t>(idx)].culling = culling;
}

void RenderEntityRegistry::SetSphere(const RenderEntityId id, const SpherePrimitive& sphere) {
  const int idx = IndexOf(id);
  if (idx < 0) {
    return;
  }
  auto primitive = sphere;
  primitive.pass = entities_[static_cast<size_t>(idx)].pass;
  entities_[static_cast<size_t>(idx)].payload = primitive;
}

void RenderEntityRegistry::SetBox(const RenderEntityId id, const BoxPrimitive& box) {
  const int idx = IndexOf(id);
  if (idx < 0) {
    return;
  }
  auto primitive = box;
  primitive.pass = entities_[static_cast<size_t>(idx)].pass;
  entities_[static_cast<size_t>(idx)].payload = primitive;
}

void RenderEntityRegistry::SetLine(const RenderEntityId id, const LinePrimitive& line) {
  const int idx = IndexOf(id);
  if (idx < 0) {
    return;
  }
  auto primitive = line;
  primitive.pass = entities_[static_cast<size_t>(idx)].pass;
  entities_[static_cast<size_t>(idx)].payload = primitive;
}

void RenderEntityRegistry::SetMesh(const RenderEntityId id, const MeshInstancePrimitive& mesh) {
  const int idx = IndexOf(id);
  if (idx < 0) {
    return;
  }
  auto primitive = mesh;
  primitive.pass = entities_[static_cast<size_t>(idx)].pass;
  entities_[static_cast<size_t>(idx)].payload = primitive;
}

void RenderEntityRegistry::SetOverlay(const RenderEntityId id, const OverlayLine& overlay) {
  const int idx = IndexOf(id);
  if (idx < 0) {
    return;
  }
  entities_[static_cast<size_t>(idx)].pass = RenderPass::Overlay;
  entities_[static_cast<size_t>(idx)].payload = overlay;
}

void RenderEntityRegistry::Clear() {
  ids_.clear();
  entities_.clear();
}

std::vector<RenderEntity> RenderEntityRegistry::BuildEntities() const {
  return entities_;
}

int RenderEntityRegistry::IndexOf(const RenderEntityId id) const {
  for (size_t i = 0; i < ids_.size(); ++i) {
    if (ids_[i] == id) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

}  // namespace repulsor3d
