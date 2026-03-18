#pragma once

#if defined(REPULSOR_HAS_NTCORE)

#include <memory>
#include <string>
#include <unordered_map>

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableType.h>
#include <networktables/StringTopic.h>

namespace repulsor3d::nt {

class SubscriberCollection {
 public:
  explicit SubscriberCollection(::nt::NetworkTableInstance* instance = nullptr);

  void SetInstance(::nt::NetworkTableInstance* instance);

  void AddDouble(const std::string& key, const std::string& topic, double defaultValue = 0.0);
  void AddBoolean(const std::string& key, const std::string& topic, bool defaultValue = false);
  void AddString(const std::string& key, const std::string& topic, std::string defaultValue = "");
  bool AddTyped(
      const std::string& key,
      const std::string& topic,
      ::nt::NetworkTableType type,
      double defaultDouble = 0.0,
      bool defaultBoolean = false,
      const std::string& defaultString = "");

  bool ContainsKey(const std::string& key) const;
  bool ContainsTopic(const std::string& topic) const;

  double GetDouble(const std::string& key, double fallback = 0.0) const;
  bool GetBoolean(const std::string& key, bool fallback = false) const;
  std::string GetString(const std::string& key, const std::string& fallback = "") const;

  void CopyFieldsWithPrefix(
      const std::string& keyPrefix,
      std::unordered_map<std::string, double>& outDoubles,
      std::unordered_map<std::string, std::string>& outStrings,
      std::unordered_map<std::string, bool>& outBooleans) const;

 private:
  ::nt::NetworkTableInstance* instance_ = nullptr;
  std::unordered_map<std::string, std::unique_ptr<::nt::DoubleSubscriber>> doubles_;
  std::unordered_map<std::string, std::unique_ptr<::nt::BooleanSubscriber>> booleans_;
  std::unordered_map<std::string, std::unique_ptr<::nt::StringSubscriber>> strings_;
  std::unordered_map<std::string, std::string> keyToTopic_;
};

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)
