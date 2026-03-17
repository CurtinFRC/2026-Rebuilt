#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "repulsor3d/render/material/Material.hpp"

namespace repulsor3d {

class MaterialPipeline {
 public:
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

 private:
  std::vector<std::unique_ptr<IMaterialPass>> passes_;
};

}  // namespace repulsor3d
