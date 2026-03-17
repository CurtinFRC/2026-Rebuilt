#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace repulsor3d {

struct RenderGraphPass {
  std::string name;
  std::vector<std::string> dependencies;
  std::function<void()> execute;
};

class RenderGraph {
 public:
  void AddPass(RenderGraphPass pass);
  void Execute() const;

 private:
  std::unordered_map<std::string, RenderGraphPass> passes_;
};

}  // namespace repulsor3d
