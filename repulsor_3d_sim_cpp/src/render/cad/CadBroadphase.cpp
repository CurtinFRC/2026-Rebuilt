#include "repulsor3d/render/cad/CadBroadphase.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_set>

namespace repulsor3d::cad {

UniformGridBroadphase::UniformGridBroadphase(const float cellSizeMeters)
    : cellSizeMeters_(std::max(0.1F, cellSizeMeters)) {}

void UniformGridBroadphase::Clear() {
  cells_.clear();
  hasCellBounds_ = false;
}

void UniformGridBroadphase::Insert(const BoundingSphereRef& sphere) {
  const float r = std::max(sphere.radius, 0.001F);
  const int minX = CellCoord(sphere.center.x - r);
  const int maxX = CellCoord(sphere.center.x + r);
  const int minY = CellCoord(sphere.center.y - r);
  const int maxY = CellCoord(sphere.center.y + r);
  const int minZ = CellCoord(sphere.center.z - r);
  const int maxZ = CellCoord(sphere.center.z + r);

  for (int x = minX; x <= maxX; ++x) {
    for (int y = minY; y <= maxY; ++y) {
      for (int z = minZ; z <= maxZ; ++z) {
        const CellKey key{x, y, z};
        cells_[key].push_back(sphere.index);
        if (!hasCellBounds_) {
          minCell_ = key;
          maxCell_ = key;
          hasCellBounds_ = true;
        } else {
          minCell_.x = std::min(minCell_.x, key.x);
          minCell_.y = std::min(minCell_.y, key.y);
          minCell_.z = std::min(minCell_.z, key.z);
          maxCell_.x = std::max(maxCell_.x, key.x);
          maxCell_.y = std::max(maxCell_.y, key.y);
          maxCell_.z = std::max(maxCell_.z, key.z);
        }
      }
    }
  }
}

void UniformGridBroadphase::QueryAabb(const Aabb& bounds, std::vector<std::size_t>& outIndices) const {
  outIndices.clear();
  if (cells_.empty() || !hasCellBounds_) {
    return;
  }
  const int minX = std::max(CellCoord(bounds.min.x), minCell_.x);
  const int maxX = std::min(CellCoord(bounds.max.x), maxCell_.x);
  const int minY = std::max(CellCoord(bounds.min.y), minCell_.y);
  const int maxY = std::min(CellCoord(bounds.max.y), maxCell_.y);
  const int minZ = std::max(CellCoord(bounds.min.z), minCell_.z);
  const int maxZ = std::min(CellCoord(bounds.max.z), maxCell_.z);
  if (minX > maxX || minY > maxY || minZ > maxZ) {
    return;
  }

  std::unordered_set<std::size_t> seen;
  seen.reserve(cells_.size() * 2U);

  const std::int64_t spanX = static_cast<std::int64_t>(maxX) - static_cast<std::int64_t>(minX) + 1;
  const std::int64_t spanY = static_cast<std::int64_t>(maxY) - static_cast<std::int64_t>(minY) + 1;
  const std::int64_t spanZ = static_cast<std::int64_t>(maxZ) - static_cast<std::int64_t>(minZ) + 1;
  const std::int64_t denseVolume = std::max<std::int64_t>(0, spanX) *
                                   std::max<std::int64_t>(0, spanY) *
                                   std::max<std::int64_t>(0, spanZ);
  constexpr std::int64_t kDenseScanCellThreshold = 200000;

  if (denseVolume > kDenseScanCellThreshold) {
    for (const auto& [key, bucket] : cells_) {
      if (key.x < minX || key.x > maxX ||
          key.y < minY || key.y > maxY ||
          key.z < minZ || key.z > maxZ) {
        continue;
      }
      for (const std::size_t index : bucket) {
        if (seen.insert(index).second) {
          outIndices.push_back(index);
        }
      }
    }
    return;
  }

  for (int x = minX; x <= maxX; ++x) {
    for (int y = minY; y <= maxY; ++y) {
      for (int z = minZ; z <= maxZ; ++z) {
        const auto it = cells_.find(CellKey{x, y, z});
        if (it == cells_.end()) {
          continue;
        }
        for (const std::size_t index : it->second) {
          if (seen.insert(index).second) {
            outIndices.push_back(index);
          }
        }
      }
    }
  }
}

std::size_t UniformGridBroadphase::CellKeyHasher::operator()(const CellKey& key) const {
  const std::size_t a = static_cast<std::size_t>(key.x) * 73856093U;
  const std::size_t b = static_cast<std::size_t>(key.y) * 19349663U;
  const std::size_t c = static_cast<std::size_t>(key.z) * 83492791U;
  return a ^ b ^ c;
}

int UniformGridBroadphase::CellCoord(const float value) const {
  return static_cast<int>(std::floor(value / cellSizeMeters_));
}

}  // namespace repulsor3d::cad
