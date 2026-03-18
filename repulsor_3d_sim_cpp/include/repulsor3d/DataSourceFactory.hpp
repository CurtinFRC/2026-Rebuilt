#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/DataSource.hpp"

namespace repulsor3d {

class DataSourceRegistry {
 public:
  using Creator = std::function<std::unique_ptr<ISnapshotSource>(const ViewerConfig&)>;

  void Register(std::string name, Creator creator);
  std::unique_ptr<ISnapshotSource> Create(const std::string& name, const ViewerConfig& cfg) const;
  bool Contains(const std::string& name) const;

 private:
  std::unordered_map<std::string, Creator> creators_;
};

DataSourceRegistry CreateDefaultDataSourceRegistry();
std::unique_ptr<ISnapshotSource> CreateDataSourceFromConfig(const ViewerConfig& cfg);

}  // namespace repulsor3d

