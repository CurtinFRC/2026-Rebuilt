#pragma once

#if defined(REPULSOR_HAS_NTCORE)

#include <optional>
#include <string>
#include <vector>

#include <networktables/Topic.h>

namespace repulsor3d::nt {

std::string TrimSlashes(std::string path);
std::string TopicPrefix(const std::string& tablePath);
std::string JoinTopic(const std::string& prefix, const std::string& leaf);

class TopicPathBuilder {
 public:
  explicit TopicPathBuilder(std::string tablePath);
  const std::string& Prefix() const;
  std::string At(const std::string& relativePath) const;

 private:
  std::string prefix_;
};

std::optional<std::string> ExtractEntityIdFromTopic(
    const std::string& topicName,
    const std::string& tablePrefix,
    const std::string& idPrefix);

std::vector<std::string> ExtractEntityIds(
    const std::vector<::nt::Topic>& topics,
    const std::string& tablePrefix,
    const std::string& idPrefix);

std::optional<std::string> TopicRelativeToEntity(
    const std::string& topicName,
    const std::string& entityRootTopic);

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)
