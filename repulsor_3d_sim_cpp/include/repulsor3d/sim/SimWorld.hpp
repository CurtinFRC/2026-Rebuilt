#pragma once

#include <string>

#include "repulsor3d/Model.hpp"

namespace repulsor3d {

class ISimWorld {
 public:
  virtual ~ISimWorld() = default;

  virtual const WorldSnapshot& Snapshot() const = 0;
  virtual bool Connected() const = 0;
  virtual int PieceCount() const = 0;
  virtual const std::string& Method() const = 0;
  virtual const SnapshotBundle& AsSnapshotBundle() const = 0;
};

class SnapshotBundleSimWorld final : public ISimWorld {
 public:
  explicit SnapshotBundleSimWorld(SnapshotBundle bundle);

  const WorldSnapshot& Snapshot() const override;
  bool Connected() const override;
  int PieceCount() const override;
  const std::string& Method() const override;
  const SnapshotBundle& AsSnapshotBundle() const override;

 private:
  SnapshotBundle bundle_;
};

}  // namespace repulsor3d

