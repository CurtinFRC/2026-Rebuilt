#pragma once

#include "repulsor3d/Config.hpp"
#include "repulsor3d/DataSource.hpp"

namespace repulsor3d {

inline constexpr int kDataSourcePluginAbiVersion = 1;

using CreateDataSourcePluginFn = ISnapshotSource* (*)(const ViewerConfig&);
using DestroyDataSourcePluginFn = void (*)(ISnapshotSource*);
using QueryDataSourcePluginAbiVersionFn = int (*)();

}  // namespace repulsor3d

