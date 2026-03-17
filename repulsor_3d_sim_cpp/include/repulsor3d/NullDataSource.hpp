#pragma once

#include "repulsor3d/DataSource.hpp"

namespace repulsor3d {

class NullDataSource final : public ISnapshotSource {
 public:
  SnapshotBundle Read() override;
};

}  // namespace repulsor3d
