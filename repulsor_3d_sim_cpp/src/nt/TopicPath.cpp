#if defined(REPULSOR_HAS_NTCORE)

#include "repulsor3d/nt/TopicPath.hpp"

#include <algorithm>

namespace repulsor3d::nt {

std::string TrimSlashes(std::string path) {
  while (!path.empty() && path.front() == '/') {
    path.erase(path.begin());
  }
  while (!path.empty() && path.back() == '/') {
    path.pop_back();
  }
  return path;
}

std::string TopicPrefix(const std::string& tablePath) {
  const std::string normalized = TrimSlashes(tablePath);
  return normalized.empty() ? "/" : "/" + normalized;
}

std::string JoinTopic(const std::string& prefix, const std::string& leaf) {
  std::string p = prefix;
  while (!p.empty() && p.back() == '/') {
    p.pop_back();
  }

  std::string l = leaf;
  while (!l.empty() && l.front() == '/') {
    l.erase(l.begin());
  }

  if (p.empty()) {
    return "/" + l;
  }
  return p + "/" + l;
}

TopicPathBuilder::TopicPathBuilder(std::string tablePath) : prefix_(TopicPrefix(tablePath)) {}

const std::string& TopicPathBuilder::Prefix() const {
  return prefix_;
}

std::string TopicPathBuilder::At(const std::string& relativePath) const {
  return JoinTopic(prefix_, relativePath);
}

std::optional<std::string> ExtractEntityIdFromTopic(
    const std::string& topicName,
    const std::string& tablePrefix,
    const std::string& idPrefix) {
  std::string normalizedTablePrefix = tablePrefix;
  while (!normalizedTablePrefix.empty() && normalizedTablePrefix.back() == '/') {
    normalizedTablePrefix.pop_back();
  }
  const std::string tablePrefixWithSlash = normalizedTablePrefix + "/";
  if (topicName.rfind(tablePrefixWithSlash, 0) != 0) {
    return std::nullopt;
  }

  const std::string relative = topicName.substr(tablePrefixWithSlash.size());
  const auto slashPos = relative.find('/');
  const std::string head = (slashPos == std::string::npos) ? relative : relative.substr(0, slashPos);
  if (head.rfind(idPrefix, 0) != 0) {
    return std::nullopt;
  }
  return head.substr(idPrefix.size());
}

std::vector<std::string> ExtractEntityIds(
    const std::vector<::nt::Topic>& topics,
    const std::string& tablePrefix,
    const std::string& idPrefix) {
  std::vector<std::string> ids;
  ids.reserve(topics.size());
  for (const auto& topic : topics) {
    if (const auto maybeId = ExtractEntityIdFromTopic(topic.GetName(), tablePrefix, idPrefix); maybeId.has_value()) {
      ids.push_back(*maybeId);
    }
  }
  std::sort(ids.begin(), ids.end());
  ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  return ids;
}

std::optional<std::string> TopicRelativeToEntity(
    const std::string& topicName,
    const std::string& entityRootTopic) {
  if (topicName == entityRootTopic) {
    return std::string{};
  }
  const std::string prefix = entityRootTopic + "/";
  if (topicName.rfind(prefix, 0) != 0) {
    return std::nullopt;
  }
  return topicName.substr(prefix.size());
}

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)
