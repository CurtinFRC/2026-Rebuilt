#include "repulsor3d/render/RenderGraph.hpp"

#include <algorithm>
#include <iostream>
#include <unordered_set>

namespace repulsor3d {

void RenderGraph::AddPass(RenderGraphPass pass) {
  if (pass.name.empty() || !pass.execute) {
    return;
  }
  passes_[pass.name] = std::move(pass);
}

void RenderGraph::Execute() const {
  enum class VisitState { kUnvisited, kVisiting, kVisited };
  std::unordered_map<std::string, VisitState> visited;
  visited.reserve(passes_.size());

  std::vector<std::string> order;
  order.reserve(passes_.size());

  const std::function<void(const std::string&)> dfs = [&](const std::string& name) {
    const auto stateIt = visited.find(name);
    if (stateIt != visited.end()) {
      if (stateIt->second == VisitState::kVisiting) {
        std::cerr << "RenderGraph cycle detected at pass: " << name << "\n";
      }
      return;
    }

    const auto passIt = passes_.find(name);
    if (passIt == passes_.end()) {
      return;
    }

    visited[name] = VisitState::kVisiting;
    for (const auto& dep : passIt->second.dependencies) {
      dfs(dep);
    }
    visited[name] = VisitState::kVisited;
    order.push_back(name);
  };

  for (const auto& [name, _] : passes_) {
    dfs(name);
  }

  std::unordered_set<std::string> executed;
  executed.reserve(order.size());
  for (const auto& name : order) {
    if (executed.contains(name)) {
      continue;
    }
    const auto passIt = passes_.find(name);
    if (passIt != passes_.end() && passIt->second.execute) {
      passIt->second.execute();
      executed.insert(name);
    }
  }
}

}  // namespace repulsor3d
