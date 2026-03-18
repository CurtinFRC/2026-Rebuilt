#pragma once

#if defined(REPULSOR_HAS_NTCORE)

#include <memory>
#include <utility>
#include <vector>

#include "repulsor3d/Model.hpp"
#include "repulsor3d/nt/EntityStream.hpp"
#include "repulsor3d/nt/TopicSchema.hpp"

namespace repulsor3d::nt {

class IWorldSnapshotAppender {
 public:
  virtual ~IWorldSnapshotAppender() = default;
  virtual void Discover() = 0;
  virtual void Append(WorldSnapshot& snapshot) const = 0;
};

template <typename TObject>
class VectorChannelAppender final : public IWorldSnapshotAppender {
 public:
  using TargetMember = std::vector<TObject> WorldSnapshot::*;

  VectorChannelAppender(
      ::nt::NetworkTableInstance* instance,
      EntityGroupSchema schema,
      typename EntityStream<TObject>::Mapper mapper,
      TargetMember targetMember)
      : stream_(instance, std::move(schema), std::move(mapper)), targetMember_(targetMember) {}

  void Discover() override {
    stream_.Discover();
  }

  void Append(WorldSnapshot& snapshot) const override {
    stream_.AppendTo(snapshot.*targetMember_);
  }

 private:
  EntityStream<TObject> stream_;
  TargetMember targetMember_;
};

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)

