#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace repulsor3d {

struct FieldVisionObject {
  std::string oid;
  std::string type;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

struct RepulsorVisionObstacle {
  std::string oid;
  std::string kind;
  double x = 0.0;
  double y = 0.0;
  double sizeX = 0.0;
  double sizeY = 0.0;
};

struct CameraInfo {
  std::string name;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yawDeg = 0.0;
  double pitchDeg = 0.0;
  double rollDeg = 0.0;
  double hfovDeg = 0.0;
  double vfovDeg = 0.0;
  double maxRange = 0.0;
};

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double thetaRad = 0.0;
};

struct WorldSnapshot {
  std::vector<FieldVisionObject> fieldVision;
  std::vector<RepulsorVisionObstacle> repulsorVision;
  std::vector<CameraInfo> cameras;
  std::vector<FieldVisionObject> truth;

  std::optional<Pose2D> pose;
  std::array<double, 6> extrinsics{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::optional<Pose2D> activeGoal;
  std::optional<Pose2D> chosenCollect;
  std::optional<Pose2D> finalCollect;
};

struct SnapshotBundle {
  WorldSnapshot snapshot;
  bool connected = false;
  int pieces = 0;
  std::string method = "N/A";
};

}  // namespace repulsor3d
