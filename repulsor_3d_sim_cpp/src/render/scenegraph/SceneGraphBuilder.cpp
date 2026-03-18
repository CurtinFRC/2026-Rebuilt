#include "repulsor3d/render/scenegraph/SceneGraphBuilder.hpp"

#include <utility>

namespace repulsor3d::scenegraph {

RenderEntity SceneGraphBuilder::AddNode(RenderEntity node) {
  node.id = EnsureNodeId(std::move(node.id));
  nodes_.push_back(std::move(node));
  return nodes_.back();
}

RenderEntity SceneGraphBuilder::AddNode(
    std::string id,
    const RenderPass pass,
    RenderEntityPayload payload,
    std::string parentId,
    Transform3D transform,
    const bool hasTransform,
    const EntityCulling culling) {
  RenderEntity node;
  node.id = std::move(id);
  node.pass = pass;
  node.payload = std::move(payload);
  node.parentId = std::move(parentId);
  node.transform = transform;
  node.hasTransform = hasTransform;
  node.culling = culling;
  return AddNode(std::move(node));
}

std::vector<RenderEntity> SceneGraphBuilder::ConsumeNodes() {
  std::vector<RenderEntity> out;
  out.swap(nodes_);
  return out;
}

std::string SceneGraphBuilder::EnsureNodeId(std::string id) {
  if (!id.empty()) {
    return id;
  }
  return "scene_node_" + std::to_string(autoIdCounter_++);
}

}  // namespace repulsor3d::scenegraph

