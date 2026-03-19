#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

#include <glm/vec3.hpp>

#include "repulsor3d/render/cad/CadFrustum.hpp"

namespace repulsor3d::cad {

struct BoundingSphereRef {
  glm::vec3 center{0.0F, 0.0F, 0.0F};
  float radius = 0.0F;
  std::size_t index = 0;
};

class UniformGridBroadphase {
 public:
  explicit UniformGridBroadphase(float cellSizeMeters = 3.0F);

  void Clear();
  void Insert(const BoundingSphereRef& sphere);
  void QueryAabb(const Aabb& bounds, std::vector<std::size_t>& outIndices) const;

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

  int CellCoord(float value) const;

  float cellSizeMeters_ = 3.0F;
  std::unordered_map<CellKey, std::vector<std::size_t>, CellKeyHasher> cells_;
  bool hasCellBounds_ = false;
  CellKey minCell_{};
  CellKey maxCell_{};
};

}  // namespace repulsor3d::cad
