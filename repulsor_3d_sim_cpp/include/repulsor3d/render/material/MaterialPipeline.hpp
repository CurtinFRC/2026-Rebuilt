#pragma once

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "repulsor3d/render/material/Material.hpp"

namespace repulsor3d {

class MaterialPipeline {
 public:
  void SetResolver(std::shared_ptr<IMaterialResolver> resolver) { resolver_ = std::move(resolver); }

  void AddPass(std::unique_ptr<IMaterialPass> pass) {
    if (pass) {
      passes_.push_back(std::move(pass));
    }
  }

  void Apply(const MaterialInstance& material) {
    for (auto& pass : passes_) {
      pass->Apply(material);
    }
  }

  void ApplyTemplate(const MaterialTemplate& materialTemplate) {
    if (resolver_ == nullptr) {
      Apply(MaterialInstance{.templateId = materialTemplate.id, .definition = materialTemplate.defaults});
      return;
    }
    Apply(resolver_->Resolve(materialTemplate));
  }

 private:
  std::shared_ptr<IMaterialResolver> resolver_;
  std::vector<std::unique_ptr<IMaterialPass>> passes_;
};

}  // namespace repulsor3d
