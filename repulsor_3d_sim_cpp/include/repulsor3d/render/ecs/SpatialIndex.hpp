#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

#include <glm/vec3.hpp>

namespace repulsor3d {

struct SpatialSphere {
  glm::vec3 center{0.0F, 0.0F, 0.0F};
  float radius = 0.0F;
  std::size_t entityIndex = 0;
};

struct SpatialAabb {
  glm::vec3 min{0.0F, 0.0F, 0.0F};
  glm::vec3 max{0.0F, 0.0F, 0.0F};
};

class UniformGridSpatialIndex {
 public:
  explicit UniformGridSpatialIndex(float cellSizeMeters = 2.0F);

  void Clear();
  void Insert(const SpatialSphere& sphere);
  void QueryAabb(const SpatialAabb& bounds, std::vector<std::size_t>& outEntityIndices) const;

 private:
  struct CellKey {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const CellKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct CellKeyHasher {
    std::size_t operator()(const CellKey& key) const;
  };

  int CellCoord(float v) const;

  float cellSizeMeters_ = 2.0F;
  std::unordered_map<CellKey, std::vector<std::size_t>, CellKeyHasher> cells_;
};

}  // namespace repulsor3d
