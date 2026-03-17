#include "repulsor3d/NullDataSource.hpp"

namespace repulsor3d {

SnapshotBundle NullDataSource::Read() {
  SnapshotBundle bundle;
  bundle.connected = false;
  bundle.pieces = 0;
  bundle.method = "N/A";
  return bundle;
}

}  // namespace repulsor3d
