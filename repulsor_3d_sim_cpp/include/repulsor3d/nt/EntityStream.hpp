#pragma once

#if defined(REPULSOR_HAS_NTCORE)

#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "repulsor3d/nt/DynamicEntityRegistry.hpp"
#include "repulsor3d/nt/SubscriberCollection.hpp"
#include "repulsor3d/nt/TopicSchema.hpp"

namespace repulsor3d::nt {

template <typename TObject>
class EntityStream {
 public:
  using Mapper = std::function<std::optional<TObject>(const std::string&, const SubscriberCollection&)>;

  EntityStream(::nt::NetworkTableInstance* instance, EntityGroupSchema schema, Mapper mapper)
      : registry_(instance, std::move(schema)), mapper_(std::move(mapper)) {}

  void Discover() {
    registry_.Discover();
  }

  void AppendTo(std::vector<TObject>& out) const {
    for (const auto& [id, subs] : registry_.Entities()) {
      if (auto mapped = mapper_(id, subs); mapped.has_value()) {
        out.push_back(std::move(*mapped));
      }
    }
  }

 private:
  DynamicEntityRegistry registry_;
  Mapper mapper_;
};

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)
