#pragma once

#include "repulsor3d/DataSource.hpp"

#if defined(REPULSOR_HAS_NTCORE)

#include <memory>

#include "repulsor3d/Config.hpp"

namespace repulsor3d {

class NtDataSource final : public ISnapshotSource {
 public:
  explicit NtDataSource(const ViewerConfig& cfg);
  ~NtDataSource() override;

  SnapshotBundle Read() override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace repulsor3d

#endif  // defined(REPULSOR_HAS_NTCORE)
