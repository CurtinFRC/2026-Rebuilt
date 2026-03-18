#include "repulsor3d/SnapshotIO.hpp"

#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>

namespace repulsor3d {
namespace {

template <typename T>
void ReadMap(
    const nlohmann::json& root,
    const char* key,
    std::unordered_map<std::string, T>& outMap) {
  outMap.clear();
  const auto it = root.find(key);
  if (it == root.end() || !it->is_object()) {
    return;
  }
  for (auto entry = it->begin(); entry != it->end(); ++entry) {
    if constexpr (std::is_same_v<T, double>) {
      if (entry.value().is_number()) {
        outMap.emplace(entry.key(), entry.value().get<double>());
      }
    } else if constexpr (std::is_same_v<T, bool>) {
      if (entry.value().is_boolean()) {
        outMap.emplace(entry.key(), entry.value().get<bool>());
      }
    } else if constexpr (std::is_same_v<T, std::string>) {
      if (entry.value().is_string()) {
        outMap.emplace(entry.key(), entry.value().get<std::string>());
      }
    }
  }
}

template <typename TObject>
nlohmann::json WriteExtras(const TObject& object) {
  return nlohmann::json{
      {"extraDoubles", object.extraDoubles},
      {"extraStrings", object.extraStrings},
      {"extraBooleans", object.extraBooleans},
  };
}

nlohmann::json PoseToJson(const std::optional<Pose2D>& pose) {
  if (!pose.has_value()) {
    return nullptr;
  }
  return nlohmann::json{{"x", pose->x}, {"y", pose->y}, {"theta", pose->thetaRad}};
}

std::optional<Pose2D> PoseFromJson(const nlohmann::json& value) {
  if (value.is_null() || !value.is_object()) {
    return std::nullopt;
  }
  Pose2D out;
  out.x = value.value("x", 0.0);
  out.y = value.value("y", 0.0);
  out.thetaRad = value.value("theta", 0.0);
  return out;
}

}  // namespace

nlohmann::json SnapshotBundleToJson(const SnapshotBundle& bundle) {
  nlohmann::json root;
  root["connected"] = bundle.connected;
  root["pieces"] = bundle.pieces;
  root["method"] = bundle.method;

  root["pose"] = PoseToJson(bundle.snapshot.pose);
  root["activeGoal"] = PoseToJson(bundle.snapshot.activeGoal);
  root["chosenCollect"] = PoseToJson(bundle.snapshot.chosenCollect);
  root["finalCollect"] = PoseToJson(bundle.snapshot.finalCollect);
  root["extrinsics"] = bundle.snapshot.extrinsics;

  root["fieldVision"] = nlohmann::json::array();
  for (const auto& object : bundle.snapshot.fieldVision) {
    nlohmann::json item{
        {"oid", object.oid},
        {"type", object.type},
        {"x", object.x},
        {"y", object.y},
        {"z", object.z},
        {"roll", object.roll},
        {"pitch", object.pitch},
        {"yaw", object.yaw},
    };
    item.update(WriteExtras(object));
    root["fieldVision"].push_back(std::move(item));
  }

  root["repulsorVision"] = nlohmann::json::array();
  for (const auto& object : bundle.snapshot.repulsorVision) {
    nlohmann::json item{
        {"oid", object.oid},
        {"kind", object.kind},
        {"x", object.x},
        {"y", object.y},
        {"sizeX", object.sizeX},
        {"sizeY", object.sizeY},
    };
    item.update(WriteExtras(object));
    root["repulsorVision"].push_back(std::move(item));
  }

  root["cameras"] = nlohmann::json::array();
  for (const auto& camera : bundle.snapshot.cameras) {
    nlohmann::json item{
        {"name", camera.name},
        {"x", camera.x},
        {"y", camera.y},
        {"z", camera.z},
        {"yawDeg", camera.yawDeg},
        {"pitchDeg", camera.pitchDeg},
        {"rollDeg", camera.rollDeg},
        {"hfovDeg", camera.hfovDeg},
        {"vfovDeg", camera.vfovDeg},
        {"maxRange", camera.maxRange},
    };
    item.update(WriteExtras(camera));
    root["cameras"].push_back(std::move(item));
  }

  root["truth"] = nlohmann::json::array();
  for (const auto& object : bundle.snapshot.truth) {
    nlohmann::json item{
        {"oid", object.oid},
        {"type", object.type},
        {"x", object.x},
        {"y", object.y},
        {"z", object.z},
        {"roll", object.roll},
        {"pitch", object.pitch},
        {"yaw", object.yaw},
    };
    item.update(WriteExtras(object));
    root["truth"].push_back(std::move(item));
  }

  root["dynamicEntityGroups"] = nlohmann::json::object();
  for (const auto& [groupName, records] : bundle.snapshot.dynamicEntityGroups) {
    nlohmann::json group = nlohmann::json::array();
    for (const auto& record : records) {
      group.push_back(nlohmann::json{
          {"id", record.id},
          {"doubles", record.doubles},
          {"strings", record.strings},
          {"booleans", record.booleans},
      });
    }
    root["dynamicEntityGroups"][groupName] = std::move(group);
  }

  return root;
}

bool SnapshotBundleFromJson(const nlohmann::json& jsonValue, SnapshotBundle& outBundle, std::string* error) {
  if (!jsonValue.is_object()) {
    if (error != nullptr) {
      *error = "snapshot JSON root must be an object";
    }
    return false;
  }

  outBundle.connected = jsonValue.value("connected", false);
  outBundle.pieces = jsonValue.value("pieces", 0);
  outBundle.method = jsonValue.value("method", std::string("N/A"));

  outBundle.snapshot.pose = PoseFromJson(jsonValue.value("pose", nullptr));
  outBundle.snapshot.activeGoal = PoseFromJson(jsonValue.value("activeGoal", nullptr));
  outBundle.snapshot.chosenCollect = PoseFromJson(jsonValue.value("chosenCollect", nullptr));
  outBundle.snapshot.finalCollect = PoseFromJson(jsonValue.value("finalCollect", nullptr));

  const auto extrinsicsIt = jsonValue.find("extrinsics");
  if (extrinsicsIt != jsonValue.end() && extrinsicsIt->is_array() && extrinsicsIt->size() == 6) {
    for (std::size_t i = 0; i < 6; ++i) {
      outBundle.snapshot.extrinsics[i] = (*extrinsicsIt)[i].get<double>();
    }
  }

  outBundle.snapshot.fieldVision.clear();
  for (const auto& item : jsonValue.value("fieldVision", nlohmann::json::array())) {
    if (!item.is_object()) {
      continue;
    }
    FieldVisionObject object;
    object.oid = item.value("oid", std::string{});
    object.type = item.value("type", std::string{});
    object.x = item.value("x", 0.0);
    object.y = item.value("y", 0.0);
    object.z = item.value("z", 0.0);
    object.roll = item.value("roll", 0.0);
    object.pitch = item.value("pitch", 0.0);
    object.yaw = item.value("yaw", 0.0);
    ReadMap(item, "extraDoubles", object.extraDoubles);
    ReadMap(item, "extraStrings", object.extraStrings);
    ReadMap(item, "extraBooleans", object.extraBooleans);
    outBundle.snapshot.fieldVision.push_back(std::move(object));
  }

  outBundle.snapshot.repulsorVision.clear();
  for (const auto& item : jsonValue.value("repulsorVision", nlohmann::json::array())) {
    if (!item.is_object()) {
      continue;
    }
    RepulsorVisionObstacle object;
    object.oid = item.value("oid", std::string{});
    object.kind = item.value("kind", std::string{});
    object.x = item.value("x", 0.0);
    object.y = item.value("y", 0.0);
    object.sizeX = item.value("sizeX", 0.0);
    object.sizeY = item.value("sizeY", 0.0);
    ReadMap(item, "extraDoubles", object.extraDoubles);
    ReadMap(item, "extraStrings", object.extraStrings);
    ReadMap(item, "extraBooleans", object.extraBooleans);
    outBundle.snapshot.repulsorVision.push_back(std::move(object));
  }

  outBundle.snapshot.cameras.clear();
  for (const auto& item : jsonValue.value("cameras", nlohmann::json::array())) {
    if (!item.is_object()) {
      continue;
    }
    CameraInfo camera;
    camera.name = item.value("name", std::string{});
    camera.x = item.value("x", 0.0);
    camera.y = item.value("y", 0.0);
    camera.z = item.value("z", 0.0);
    camera.yawDeg = item.value("yawDeg", 0.0);
    camera.pitchDeg = item.value("pitchDeg", 0.0);
    camera.rollDeg = item.value("rollDeg", 0.0);
    camera.hfovDeg = item.value("hfovDeg", 0.0);
    camera.vfovDeg = item.value("vfovDeg", 0.0);
    camera.maxRange = item.value("maxRange", 0.0);
    ReadMap(item, "extraDoubles", camera.extraDoubles);
    ReadMap(item, "extraStrings", camera.extraStrings);
    ReadMap(item, "extraBooleans", camera.extraBooleans);
    outBundle.snapshot.cameras.push_back(std::move(camera));
  }

  outBundle.snapshot.truth.clear();
  for (const auto& item : jsonValue.value("truth", nlohmann::json::array())) {
    if (!item.is_object()) {
      continue;
    }
    FieldVisionObject object;
    object.oid = item.value("oid", std::string{});
    object.type = item.value("type", std::string{});
    object.x = item.value("x", 0.0);
    object.y = item.value("y", 0.0);
    object.z = item.value("z", 0.0);
    object.roll = item.value("roll", 0.0);
    object.pitch = item.value("pitch", 0.0);
    object.yaw = item.value("yaw", 0.0);
    ReadMap(item, "extraDoubles", object.extraDoubles);
    ReadMap(item, "extraStrings", object.extraStrings);
    ReadMap(item, "extraBooleans", object.extraBooleans);
    outBundle.snapshot.truth.push_back(std::move(object));
  }

  outBundle.snapshot.dynamicEntityGroups.clear();
  const auto dynamicGroupsIt = jsonValue.find("dynamicEntityGroups");
  if (dynamicGroupsIt != jsonValue.end() && dynamicGroupsIt->is_object()) {
    for (auto it = dynamicGroupsIt->begin(); it != dynamicGroupsIt->end(); ++it) {
      if (!it.value().is_array()) {
        continue;
      }
      std::vector<DynamicEntityRecord> records;
      records.reserve(it.value().size());
      for (const auto& item : it.value()) {
        if (!item.is_object()) {
          continue;
        }
        DynamicEntityRecord record;
        record.id = item.value("id", std::string{});
        ReadMap(item, "doubles", record.doubles);
        ReadMap(item, "strings", record.strings);
        ReadMap(item, "booleans", record.booleans);
        records.push_back(std::move(record));
      }
      outBundle.snapshot.dynamicEntityGroups.emplace(it.key(), std::move(records));
    }
  }

  return true;
}

}  // namespace repulsor3d
