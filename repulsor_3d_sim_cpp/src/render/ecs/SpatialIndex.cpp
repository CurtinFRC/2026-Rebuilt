#include "repulsor3d/render/ecs/SpatialIndex.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <unordered_set>

namespace repulsor3d {

UniformGridSpatialIndex::UniformGridSpatialIndex(const float cellSizeMeters)
    : cellSizeMeters_(std::max(0.1F, cellSizeMeters)) {}

void UniformGridSpatialIndex::Clear() {
  cells_.clear();
}

void UniformGridSpatialIndex::Insert(const SpatialSphere& sphere) {
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
        cells_[CellKey{x, y, z}].push_back(sphere.entityIndex);
      }
    }
  }
}

void UniformGridSpatialIndex::QueryAabb(const SpatialAabb& bounds, std::vector<std::size_t>& outEntityIndices) const {
  outEntityIndices.clear();
  if (cells_.empty()) {
    return;
  }

  const int minX = CellCoord(bounds.min.x);
  const int maxX = CellCoord(bounds.max.x);
  const int minY = CellCoord(bounds.min.y);
  const int maxY = CellCoord(bounds.max.y);
  const int minZ = CellCoord(bounds.min.z);
  const int maxZ = CellCoord(bounds.max.z);

  const std::int64_t spanX = static_cast<std::int64_t>(maxX) - static_cast<std::int64_t>(minX) + 1LL;
  const std::int64_t spanY = static_cast<std::int64_t>(maxY) - static_cast<std::int64_t>(minY) + 1LL;
  const std::int64_t spanZ = static_cast<std::int64_t>(maxZ) - static_cast<std::int64_t>(minZ) + 1LL;
  constexpr std::int64_t kMaxAxisSpan = 384;
  constexpr std::int64_t kMaxQueryVolume = 4'000'000;
  const bool pathologicalRange =
      spanX <= 0 || spanY <= 0 || spanZ <= 0 ||
      spanX > kMaxAxisSpan || spanY > kMaxAxisSpan || spanZ > kMaxAxisSpan ||
      (spanX * spanY * spanZ) > kMaxQueryVolume;

  std::unordered_set<std::size_t> dedup;
  dedup.reserve(cells_.size() * 2U);
  if (pathologicalRange) {
    for (const auto& [key, entityIndices] : cells_) {
      (void)key;
      for (const std::size_t idx : entityIndices) {
        if (dedup.insert(idx).second) {
          outEntityIndices.push_back(idx);
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
        for (const std::size_t idx : it->second) {
          if (dedup.insert(idx).second) {
            outEntityIndices.push_back(idx);
          }
        }
      }
    }
  }
}

std::size_t UniformGridSpatialIndex::CellKeyHasher::operator()(const CellKey& key) const {
  const std::size_t a = static_cast<std::size_t>(key.x) * 73856093U;
  const std::size_t b = static_cast<std::size_t>(key.y) * 19349663U;
  const std::size_t c = static_cast<std::size_t>(key.z) * 83492791U;
  return a ^ b ^ c;
}

int UniformGridSpatialIndex::CellCoord(const float v) const {
  return static_cast<int>(std::floor(v / cellSizeMeters_));
}

}  // namespace repulsor3d
