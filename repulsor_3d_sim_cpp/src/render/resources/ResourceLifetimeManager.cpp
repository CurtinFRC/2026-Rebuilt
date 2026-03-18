#include "repulsor3d/render/resources/ResourceLifetimeManager.hpp"

#include <algorithm>

namespace repulsor3d {

void ResourceLifetimeManager::SetBudget(const ResourceClass resourceClass, const ResourceBudget budget) {
  budgets_[resourceClass] = budget;
}

void ResourceLifetimeManager::Register(const ResourceClass resourceClass, const std::size_t bytes) {
  auto& stats = stats_[resourceClass];
  stats.count += 1;
  stats.bytes += bytes;
}

void ResourceLifetimeManager::Release(const ResourceClass resourceClass, const std::size_t bytes) {
  auto& stats = stats_[resourceClass];
  if (stats.count > 0) {
    stats.count -= 1;
  }
  if (stats.bytes > bytes) {
    stats.bytes -= bytes;
  } else {
    stats.bytes = 0;
  }
}

ResourceStats ResourceLifetimeManager::Stats(const ResourceClass resourceClass) const {
  const auto it = stats_.find(resourceClass);
  if (it == stats_.end()) {
    return {};
  }
  return it->second;
}

bool ResourceLifetimeManager::IsOverBudget(const ResourceClass resourceClass) const {
  const auto budgetIt = budgets_.find(resourceClass);
  if (budgetIt == budgets_.end()) {
    return false;
  }

  const auto stats = Stats(resourceClass);
  const ResourceBudget& budget = budgetIt->second;
  const bool overCount = budget.maxCount > 0 && stats.count > budget.maxCount;
  const bool overBytes = budget.maxBytes > 0 && stats.bytes > budget.maxBytes;
  return overCount || overBytes;
}

}  // namespace repulsor3d
