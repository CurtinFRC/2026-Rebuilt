#if defined(REPULSOR_HAS_NTCORE)

#include "repulsor3d/nt/DynamicEntityRegistry.hpp"

#include <algorithm>
#include <string>

#include "repulsor3d/nt/TopicPath.hpp"

namespace repulsor3d::nt {
namespace {

std::string ToExtraKey(const std::string& extraPrefix, std::string suffix) {
  std::replace(suffix.begin(), suffix.end(), '/', '.');
  return extraPrefix + suffix;
}

}  // namespace

DynamicEntityRegistry::DynamicEntityRegistry(::nt::NetworkTableInstance* instance, EntityGroupSchema schema)
    : instance_(instance), schema_(std::move(schema)) {}

void DynamicEntityRegistry::Discover() {
  if (instance_ == nullptr) {
    return;
  }

  const std::vector<::nt::Topic> topics = instance_->GetTopics(schema_.tablePrefix);
  const std::vector<std::string> ids = ExtractEntityIds(topics, schema_.tablePrefix, schema_.idPrefix);

  for (const auto& id : ids) {
    auto [it, inserted] = entities_.try_emplace(id, instance_);
    if (inserted) {
      EnsureEntityKnownFields(id, it->second);
    }
    if (schema_.autoCaptureAdditionalScalars) {
      CaptureAdditionalEntityFields(id, it->second, topics);
    }
  }
}

const std::unordered_map<std::string, SubscriberCollection>& DynamicEntityRegistry::Entities() const {
  return entities_;
}

void DynamicEntityRegistry::EnsureEntityKnownFields(const std::string& entityId, SubscriberCollection& subs) const {
  const std::string entityRoot = JoinTopic(schema_.tablePrefix, schema_.idPrefix + entityId);
  const auto subscribeSpec = [&](const ScalarFieldSpec& spec) {
    const std::string topic = spec.topicSuffix.empty() ? entityRoot : JoinTopic(entityRoot, spec.topicSuffix);
    subs.AddTyped(spec.key, topic, spec.type, spec.defaultDouble, spec.defaultBoolean, spec.defaultString);
  };

  subscribeSpec(schema_.aliveField);
  for (const auto& spec : schema_.fields) {
    subscribeSpec(spec);
  }
}

void DynamicEntityRegistry::CaptureAdditionalEntityFields(
    const std::string& entityId,
    SubscriberCollection& subs,
    const std::vector<::nt::Topic>& topics) const {
  const std::string entityRoot = JoinTopic(schema_.tablePrefix, schema_.idPrefix + entityId);

  for (const auto& topic : topics) {
    const std::string topicName = topic.GetName();
    const auto maybeRelative = TopicRelativeToEntity(topicName, entityRoot);
    if (!maybeRelative.has_value() || maybeRelative->empty()) {
      continue;
    }
    if (subs.ContainsTopic(topicName)) {
      continue;
    }

    const std::string key = ToExtraKey(schema_.extraKeyPrefix, *maybeRelative);
    if (subs.ContainsKey(key)) {
      continue;
    }

    const ::nt::NetworkTableType type = topic.GetType();
    subs.AddTyped(key, topicName, type);
  }
}

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)
