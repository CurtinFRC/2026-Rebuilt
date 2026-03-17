#pragma once

#include "repulsor3d/Model.hpp"

namespace repulsor3d {

class ISnapshotSource {
 public:
  virtual ~ISnapshotSource() = default;
  virtual SnapshotBundle Read() = 0;
};

}  // namespace repulsor3d
