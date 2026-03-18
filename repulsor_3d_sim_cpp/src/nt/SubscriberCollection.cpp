#if defined(REPULSOR_HAS_NTCORE)

#include "repulsor3d/nt/SubscriberCollection.hpp"

namespace repulsor3d::nt {

SubscriberCollection::SubscriberCollection(::nt::NetworkTableInstance* instance) : instance_(instance) {}

void SubscriberCollection::SetInstance(::nt::NetworkTableInstance* instance) {
  instance_ = instance;
}

void SubscriberCollection::AddDouble(const std::string& key, const std::string& topic, const double defaultValue) {
  if (instance_ == nullptr) {
    return;
  }
  doubles_[key] = std::make_unique<::nt::DoubleSubscriber>(instance_->GetDoubleTopic(topic).Subscribe(defaultValue));
  keyToTopic_[key] = topic;
}

void SubscriberCollection::AddBoolean(const std::string& key, const std::string& topic, const bool defaultValue) {
  if (instance_ == nullptr) {
    return;
  }
  booleans_[key] = std::make_unique<::nt::BooleanSubscriber>(instance_->GetBooleanTopic(topic).Subscribe(defaultValue));
  keyToTopic_[key] = topic;
}

void SubscriberCollection::AddString(const std::string& key, const std::string& topic, std::string defaultValue) {
  if (instance_ == nullptr) {
    return;
  }
  strings_[key] = std::make_unique<::nt::StringSubscriber>(instance_->GetStringTopic(topic).Subscribe(std::move(defaultValue)));
  keyToTopic_[key] = topic;
}

bool SubscriberCollection::AddTyped(
    const std::string& key,
    const std::string& topic,
    const ::nt::NetworkTableType type,
    const double defaultDouble,
    const bool defaultBoolean,
    const std::string& defaultString) {
  switch (type) {
    case ::nt::NetworkTableType::kDouble:
      AddDouble(key, topic, defaultDouble);
      return true;
    case ::nt::NetworkTableType::kBoolean:
      AddBoolean(key, topic, defaultBoolean);
      return true;
    case ::nt::NetworkTableType::kString:
      AddString(key, topic, defaultString);
      return true;
    default:
      break;
  }
  return false;
}

bool SubscriberCollection::ContainsKey(const std::string& key) const {
  return doubles_.contains(key) || booleans_.contains(key) || strings_.contains(key);
}

bool SubscriberCollection::ContainsTopic(const std::string& topic) const {
  for (const auto& [_, knownTopic] : keyToTopic_) {
    if (knownTopic == topic) {
      return true;
    }
  }
  return false;
}

double SubscriberCollection::GetDouble(const std::string& key, const double fallback) const {
  const auto it = doubles_.find(key);
  if (it == doubles_.end() || it->second == nullptr) {
    return fallback;
  }
  return it->second->Get();
}

bool SubscriberCollection::GetBoolean(const std::string& key, const bool fallback) const {
  const auto it = booleans_.find(key);
  if (it == booleans_.end() || it->second == nullptr) {
    return fallback;
  }
  return it->second->Get();
}

std::string SubscriberCollection::GetString(const std::string& key, const std::string& fallback) const {
  const auto it = strings_.find(key);
  if (it == strings_.end() || it->second == nullptr) {
    return fallback;
  }
  return it->second->Get();
}

void SubscriberCollection::CopyFieldsWithPrefix(
    const std::string& keyPrefix,
    std::unordered_map<std::string, double>& outDoubles,
    std::unordered_map<std::string, std::string>& outStrings,
    std::unordered_map<std::string, bool>& outBooleans) const {
  const auto suffixFor = [&keyPrefix](const std::string& key) -> std::string {
    if (keyPrefix.empty() || key.rfind(keyPrefix, 0) != 0) {
      return {};
    }
    return key.substr(keyPrefix.size());
  };

  for (const auto& [key, sub] : doubles_) {
    const std::string suffix = suffixFor(key);
    if (suffix.empty() || sub == nullptr) {
      continue;
    }
    outDoubles[suffix] = sub->Get();
  }
  for (const auto& [key, sub] : strings_) {
    const std::string suffix = suffixFor(key);
    if (suffix.empty() || sub == nullptr) {
      continue;
    }
    outStrings[suffix] = sub->Get();
  }
  for (const auto& [key, sub] : booleans_) {
    const std::string suffix = suffixFor(key);
    if (suffix.empty() || sub == nullptr) {
      continue;
    }
    outBooleans[suffix] = sub->Get();
  }
}

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)
