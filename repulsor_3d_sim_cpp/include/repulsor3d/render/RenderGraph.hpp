#pragma once

#include <functional>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <vector>

namespace repulsor3d {

class RenderGraphContext {
 public:
  void ClearResources() { resources_.clear(); }

  void MarkResourceAvailable(std::string resourceName) {
    if (!resourceName.empty()) {
      resources_.insert(std::move(resourceName));
    }
  }

  bool IsResourceAvailable(const std::string& resourceName) const {
    if (resourceName.empty()) {
      return true;
    }
    return resources_.contains(resourceName);
  }

 private:
  std::unordered_set<std::string> resources_;
};

struct RenderGraphPass {
  std::string name;
  std::vector<std::string> dependencies;
  std::vector<std::string> consumesResources;
  std::vector<std::string> producesResources;
  std::function<void(RenderGraphContext&)> execute;
};

class RenderGraph {
 public:
  void AddPass(RenderGraphPass pass);
  void Execute();

 private:
  std::unordered_map<std::string, RenderGraphPass> passes_;
  RenderGraphContext context_;
};

}  // namespace repulsor3d
