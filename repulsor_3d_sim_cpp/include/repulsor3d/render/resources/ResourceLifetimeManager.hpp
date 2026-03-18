#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace repulsor3d {

enum class ResourceClass : std::uint8_t {
  Buffer,
  VertexArray,
  Texture,
  ShaderProgram,
  Mesh,
};

struct ResourceStats {
  std::size_t count = 0;
  std::size_t bytes = 0;
};

struct ResourceBudget {
  std::size_t maxCount = 0;
  std::size_t maxBytes = 0;
};

struct ResourceHandleInfo {
  std::string handleId;
  std::size_t bytes = 0;
  std::uint64_t lastTouchedTick = 0;
};

class ResourceLifetimeManager {
 public:
  struct EvictionCandidate {
    ResourceClass resourceClass = ResourceClass::Buffer;
    std::string handleId;
    std::size_t bytes = 0;
  };

  void SetBudget(ResourceClass resourceClass, ResourceBudget budget);
  void Register(ResourceClass resourceClass, std::size_t bytes = 0);
  void Register(ResourceClass resourceClass, std::string handleId, std::size_t bytes = 0);
  void Release(ResourceClass resourceClass, std::size_t bytes = 0);
  void Release(ResourceClass resourceClass, const std::string& handleId);
  void Touch(ResourceClass resourceClass, const std::string& handleId);

  ResourceStats Stats(ResourceClass resourceClass) const;
  bool IsOverBudget(ResourceClass resourceClass) const;
  std::size_t OverBudgetCount(ResourceClass resourceClass) const;
  std::size_t OverBudgetBytes(ResourceClass resourceClass) const;

  std::vector<EvictionCandidate> CollectEvictionCandidates(ResourceClass resourceClass, std::size_t maxCount = 16) const;
  bool EvictOne(ResourceClass resourceClass, const std::function<bool(const EvictionCandidate&)>& evictFn);

 private:
  std::uint64_t NextTick();

  std::unordered_map<ResourceClass, ResourceStats> stats_;
  std::unordered_map<ResourceClass, ResourceBudget> budgets_;
  std::unordered_map<ResourceClass, std::unordered_map<std::string, ResourceHandleInfo>> trackedHandles_;
  std::uint64_t touchTick_ = 1;
};

}  // namespace repulsor3d
