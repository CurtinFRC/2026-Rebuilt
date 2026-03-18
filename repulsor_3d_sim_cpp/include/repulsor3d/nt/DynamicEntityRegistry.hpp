#pragma once

#if defined(REPULSOR_HAS_NTCORE)

#include <string>
#include <unordered_map>
#include <vector>

#include <networktables/NetworkTableInstance.h>
#include <networktables/Topic.h>

#include "repulsor3d/nt/SubscriberCollection.hpp"
#include "repulsor3d/nt/TopicSchema.hpp"

namespace repulsor3d::nt {

class DynamicEntityRegistry {
 public:
  DynamicEntityRegistry(::nt::NetworkTableInstance* instance, EntityGroupSchema schema);

  void Discover();

  const std::unordered_map<std::string, SubscriberCollection>& Entities() const;

 private:
  void EnsureEntityKnownFields(const std::string& entityId, SubscriberCollection& subs) const;
  void CaptureAdditionalEntityFields(
      const std::string& entityId,
      SubscriberCollection& subs,
      const std::vector<::nt::Topic>& topics) const;

  ::nt::NetworkTableInstance* instance_ = nullptr;
  EntityGroupSchema schema_;
  std::unordered_map<std::string, SubscriberCollection> entities_;
};

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)
