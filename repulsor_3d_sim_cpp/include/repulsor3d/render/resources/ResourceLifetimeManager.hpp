#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>

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

class ResourceLifetimeManager {
 public:
  void SetBudget(ResourceClass resourceClass, ResourceBudget budget);
  void Register(ResourceClass resourceClass, std::size_t bytes = 0);
  void Release(ResourceClass resourceClass, std::size_t bytes = 0);

  ResourceStats Stats(ResourceClass resourceClass) const;
  bool IsOverBudget(ResourceClass resourceClass) const;

 private:
  std::unordered_map<ResourceClass, ResourceStats> stats_;
  std::unordered_map<ResourceClass, ResourceBudget> budgets_;
};

}  // namespace repulsor3d
