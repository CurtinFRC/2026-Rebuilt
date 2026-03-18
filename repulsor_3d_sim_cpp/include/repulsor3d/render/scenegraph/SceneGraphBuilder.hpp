#pragma once

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "repulsor3d/render/SceneFrame.hpp"

namespace repulsor3d::scenegraph {

class SceneGraphBuilder {
 public:
  SceneGraphBuilder() = default;

  RenderEntity AddNode(RenderEntity node);
  RenderEntity AddNode(
      std::string id,
      RenderPass pass,
      RenderEntityPayload payload,
      std::string parentId,
      Transform3D transform,
      bool hasTransform,
      EntityCulling culling);

  std::vector<RenderEntity> ConsumeNodes();
  bool Empty() const { return nodes_.empty(); }
  std::size_t Size() const { return nodes_.size(); }

 private:
  std::string EnsureNodeId(std::string id);

  std::vector<RenderEntity> nodes_;
  std::size_t autoIdCounter_ = 0;
};

}  // namespace repulsor3d::scenegraph

