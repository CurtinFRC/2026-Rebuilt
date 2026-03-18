#pragma once

#include <memory>
#include <string>

namespace repulsor3d {

struct RenderSceneFrame;

class IRenderableComponent {
 public:
  virtual ~IRenderableComponent() = default;
  virtual std::string Id() const = 0;
  virtual void AppendToFrame(RenderSceneFrame& frame) const = 0;
};

struct RenderableComponentBinding {
  std::string id;
  std::shared_ptr<IRenderableComponent> component;
};

}  // namespace repulsor3d
