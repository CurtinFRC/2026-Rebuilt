#if defined(REPULSOR_HAS_NTCORE)

#include "repulsor3d/nt/DomainMappers.hpp"

namespace repulsor3d::nt {

std::optional<FieldVisionObject> TryMapFieldVisionObject(const std::string& id, const SubscriberCollection& subs) {
  if (!subs.GetBoolean("alive", false)) {
    return std::nullopt;
  }
  FieldVisionObject object;
  object.oid = id;
  object.type = subs.GetString("type", "");
  object.x = subs.GetDouble("x", 0.0);
  object.y = subs.GetDouble("y", 0.0);
  object.z = subs.GetDouble("z", 0.0);
  object.roll = subs.GetDouble("roll", 0.0);
  object.pitch = subs.GetDouble("pitch", 0.0);
  object.yaw = subs.GetDouble("yaw", 0.0);
  subs.CopyFieldsWithPrefix("extra.", object.extraDoubles, object.extraStrings, object.extraBooleans);
  return object;
}

std::optional<RepulsorVisionObstacle> TryMapRepulsorObstacle(const std::string& id, const SubscriberCollection& subs) {
  if (!subs.GetBoolean("alive", false)) {
    return std::nullopt;
  }
  RepulsorVisionObstacle obstacle;
  obstacle.oid = id;
  obstacle.kind = subs.GetString("kind", "");
  obstacle.x = subs.GetDouble("x", 0.0);
  obstacle.y = subs.GetDouble("y", 0.0);
  obstacle.sizeX = subs.GetDouble("size_x", 0.0);
  obstacle.sizeY = subs.GetDouble("size_y", 0.0);
  subs.CopyFieldsWithPrefix("extra.", obstacle.extraDoubles, obstacle.extraStrings, obstacle.extraBooleans);
  return obstacle;
}

std::optional<CameraInfo> TryMapCameraInfo(const std::string& id, const SubscriberCollection& subs) {
  if (!subs.GetBoolean("alive", false)) {
    return std::nullopt;
  }
  CameraInfo camera;
  camera.name = id;
  camera.x = subs.GetDouble("x", 0.0);
  camera.y = subs.GetDouble("y", 0.0);
  camera.z = subs.GetDouble("z", 0.0);
  camera.yawDeg = subs.GetDouble("yaw_deg", 0.0);
  camera.pitchDeg = subs.GetDouble("pitch_deg", 0.0);
  camera.rollDeg = subs.GetDouble("roll_deg", 0.0);
  camera.hfovDeg = subs.GetDouble("hfov_deg", 0.0);
  camera.vfovDeg = subs.GetDouble("vfov_deg", 0.0);
  camera.maxRange = subs.GetDouble("max_range", 0.0);
  subs.CopyFieldsWithPrefix("extra.", camera.extraDoubles, camera.extraStrings, camera.extraBooleans);
  return camera;
}

std::optional<DynamicEntityRecord> TryMapDynamicEntityRecord(const std::string& id, const SubscriberCollection& subs) {
  if (!subs.GetBoolean("alive", false)) {
    return std::nullopt;
  }

  DynamicEntityRecord out;
  out.id = id;
  subs.CopyFieldsWithPrefix("", out.doubles, out.strings, out.booleans);
  out.doubles.erase("alive");
  out.strings.erase("alive");
  out.booleans.erase("alive");
  return out;
}

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)
