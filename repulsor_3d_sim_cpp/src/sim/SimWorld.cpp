#include "repulsor3d/sim/SimWorld.hpp"

#include <utility>

namespace repulsor3d {

SnapshotBundleSimWorld::SnapshotBundleSimWorld(SnapshotBundle bundle) : bundle_(std::move(bundle)) {}

const WorldSnapshot& SnapshotBundleSimWorld::Snapshot() const {
  return bundle_.snapshot;
}

bool SnapshotBundleSimWorld::Connected() const {
  return bundle_.connected;
}

int SnapshotBundleSimWorld::PieceCount() const {
  return bundle_.pieces;
}

const std::string& SnapshotBundleSimWorld::Method() const {
  return bundle_.method;
}

const SnapshotBundle& SnapshotBundleSimWorld::AsSnapshotBundle() const {
  return bundle_;
}

}  // namespace repulsor3d

