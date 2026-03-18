#include "repulsor3d/render/resources/ResourceLifetimeManager.hpp"

#include <algorithm>
#include <utility>

namespace repulsor3d {

std::uint64_t ResourceLifetimeManager::NextTick() {
  return touchTick_++;
}

void ResourceLifetimeManager::SetBudget(const ResourceClass resourceClass, const ResourceBudget budget) {
  budgets_[resourceClass] = budget;
}

void ResourceLifetimeManager::Register(const ResourceClass resourceClass, const std::size_t bytes) {
  auto& stats = stats_[resourceClass];
  stats.count += 1;
  stats.bytes += bytes;
}

void ResourceLifetimeManager::Register(
    const ResourceClass resourceClass,
    std::string handleId,
    const std::size_t bytes) {
  if (handleId.empty()) {
    Register(resourceClass, bytes);
    return;
  }

  auto& handles = trackedHandles_[resourceClass];
  auto it = handles.find(handleId);
  if (it != handles.end()) {
    auto& stats = stats_[resourceClass];
    if (stats.bytes >= it->second.bytes) {
      stats.bytes -= it->second.bytes;
    } else {
      stats.bytes = 0;
    }
    it->second.bytes = bytes;
    it->second.lastTouchedTick = NextTick();
    stats.bytes += bytes;
    return;
  }

  const std::string stableHandleId = handleId;
  handles.emplace(
      stableHandleId,
      ResourceHandleInfo{
          .handleId = stableHandleId,
          .bytes = bytes,
          .lastTouchedTick = NextTick(),
      });
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

void ResourceLifetimeManager::Release(const ResourceClass resourceClass, const std::string& handleId) {
  if (handleId.empty()) {
    Release(resourceClass, 0);
    return;
  }

  auto trackedClassIt = trackedHandles_.find(resourceClass);
  if (trackedClassIt == trackedHandles_.end()) {
    return;
  }

  auto& handles = trackedClassIt->second;
  auto it = handles.find(handleId);
  if (it == handles.end()) {
    return;
  }

  Release(resourceClass, it->second.bytes);
  handles.erase(it);
}

void ResourceLifetimeManager::Touch(const ResourceClass resourceClass, const std::string& handleId) {
  if (handleId.empty()) {
    return;
  }
  auto trackedClassIt = trackedHandles_.find(resourceClass);
  if (trackedClassIt == trackedHandles_.end()) {
    return;
  }
  auto it = trackedClassIt->second.find(handleId);
  if (it == trackedClassIt->second.end()) {
    return;
  }
  it->second.lastTouchedTick = NextTick();
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

std::size_t ResourceLifetimeManager::OverBudgetCount(const ResourceClass resourceClass) const {
  const auto budgetIt = budgets_.find(resourceClass);
  if (budgetIt == budgets_.end() || budgetIt->second.maxCount == 0) {
    return 0;
  }
  const auto stats = Stats(resourceClass);
  return stats.count > budgetIt->second.maxCount ? (stats.count - budgetIt->second.maxCount) : 0;
}

std::size_t ResourceLifetimeManager::OverBudgetBytes(const ResourceClass resourceClass) const {
  const auto budgetIt = budgets_.find(resourceClass);
  if (budgetIt == budgets_.end() || budgetIt->second.maxBytes == 0) {
    return 0;
  }
  const auto stats = Stats(resourceClass);
  return stats.bytes > budgetIt->second.maxBytes ? (stats.bytes - budgetIt->second.maxBytes) : 0;
}

std::vector<ResourceLifetimeManager::EvictionCandidate> ResourceLifetimeManager::CollectEvictionCandidates(
    const ResourceClass resourceClass,
    const std::size_t maxCount) const {
  std::vector<EvictionCandidate> out;
  const auto trackedClassIt = trackedHandles_.find(resourceClass);
  if (trackedClassIt == trackedHandles_.end() || maxCount == 0) {
    return out;
  }

  std::vector<const ResourceHandleInfo*> handles;
  handles.reserve(trackedClassIt->second.size());
  for (const auto& [_, info] : trackedClassIt->second) {
    handles.push_back(&info);
  }
  std::sort(handles.begin(), handles.end(), [](const auto* lhs, const auto* rhs) {
    return lhs->lastTouchedTick < rhs->lastTouchedTick;
  });

  const std::size_t takeCount = std::min(maxCount, handles.size());
  out.reserve(takeCount);
  for (std::size_t i = 0; i < takeCount; ++i) {
    out.push_back(EvictionCandidate{
        .resourceClass = resourceClass,
        .handleId = handles[i]->handleId,
        .bytes = handles[i]->bytes,
    });
  }
  return out;
}

bool ResourceLifetimeManager::EvictOne(
    const ResourceClass resourceClass,
    const std::function<bool(const EvictionCandidate&)>& evictFn) {
  if (!evictFn) {
    return false;
  }
  const auto candidates = CollectEvictionCandidates(resourceClass, 1);
  if (candidates.empty()) {
    return false;
  }
  if (!evictFn(candidates.front())) {
    return false;
  }
  Release(resourceClass, candidates.front().handleId);
  return true;
}

}  // namespace repulsor3d
