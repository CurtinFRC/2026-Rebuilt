#pragma once

#include <string>

#include <nlohmann/json.hpp>

#include "repulsor3d/Model.hpp"

namespace repulsor3d {

nlohmann::json SnapshotBundleToJson(const SnapshotBundle& bundle);
bool SnapshotBundleFromJson(const nlohmann::json& jsonValue, SnapshotBundle& outBundle, std::string* error = nullptr);

}  // namespace repulsor3d

