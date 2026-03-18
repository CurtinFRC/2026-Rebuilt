#pragma once

#include <algorithm>
#include <cmath>

#include <glm/trigonometric.hpp>
#include <glm/vec2.hpp>

#include "repulsor3d/Model.hpp"
#include "repulsor3d/Units.hpp"

namespace repulsor3d {

class CoordinateFrameMapper {
 public:
  struct Config {
    double originXM = 0.0;
    double originYM = 0.0;
    double rotationDeg = 0.0;
    double scaleMPerUnit = 1.0;
    double zScaleMPerUnit = 1.0;
  };

  explicit CoordinateFrameMapper(const Config& cfg) : cfg_(cfg) {
    const units::Radians rot = units::ToRadians(units::Degrees(cfg_.rotationDeg));
    const double rotRad = rot.Value();
    cosRot_ = std::cos(rotRad);
    sinRot_ = std::sin(rotRad);
  }

  glm::vec2 TransformPointXY(const double x, const double y) const {
    const units::Meters sx(x * cfg_.scaleMPerUnit);
    const units::Meters sy(y * cfg_.scaleMPerUnit);
    const double rx = sx.Value() * cosRot_ - sy.Value() * sinRot_;
    const double ry = sx.Value() * sinRot_ + sy.Value() * cosRot_;
    return glm::vec2{
        static_cast<float>(cfg_.originXM + rx),
        static_cast<float>(cfg_.originYM + ry)};
  }

  double TransformZ(const double z) const { return units::Meters(z * cfg_.zScaleMPerUnit).Value(); }

  double TransformAngleRad(const double thetaRad) const {
    const units::Radians incoming(thetaRad);
    const units::Radians rotation = units::ToRadians(units::Degrees(cfg_.rotationDeg));
    const double wrapped = incoming.Value() + rotation.Value();
    return std::atan2(std::sin(wrapped), std::cos(wrapped));
  }

  double TransformLengthXY(const double lengthUnits) const {
    return lengthUnits * cfg_.scaleMPerUnit;
  }

  double TransformLengthZ(const double lengthUnits) const {
    return lengthUnits * cfg_.zScaleMPerUnit;
  }

  Pose2D TransformPose(const Pose2D& pose) const {
    const glm::vec2 p = TransformPointXY(pose.x, pose.y);
    return Pose2D{
        .x = static_cast<double>(p.x),
        .y = static_cast<double>(p.y),
        .thetaRad = TransformAngleRad(pose.thetaRad)};
  }

  void TransformSnapshot(WorldSnapshot& snapshot) const {
    for (auto& object : snapshot.fieldVision) {
      const glm::vec2 p = TransformPointXY(object.x, object.y);
      object.x = static_cast<double>(p.x);
      object.y = static_cast<double>(p.y);
      object.z = TransformZ(object.z);
    }

    for (auto& object : snapshot.truth) {
      const glm::vec2 p = TransformPointXY(object.x, object.y);
      object.x = static_cast<double>(p.x);
      object.y = static_cast<double>(p.y);
      object.z = TransformZ(object.z);
    }

    for (auto& obstacle : snapshot.repulsorVision) {
      const glm::vec2 p = TransformPointXY(obstacle.x, obstacle.y);
      obstacle.x = static_cast<double>(p.x);
      obstacle.y = static_cast<double>(p.y);
      obstacle.sizeX = TransformLengthXY(obstacle.sizeX);
      obstacle.sizeY = TransformLengthXY(obstacle.sizeY);
    }

    for (auto& camera : snapshot.cameras) {
      // Camera offsets are local to robot frame, so only unit scaling is applied.
      camera.x = TransformLengthXY(camera.x);
      camera.y = TransformLengthXY(camera.y);
      camera.z = TransformLengthZ(camera.z);
      camera.maxRange = TransformLengthXY(camera.maxRange);
    }

    for (auto& [_, group] : snapshot.dynamicEntityGroups) {
      for (auto& record : group) {
        auto xIt = record.doubles.find("x");
        auto yIt = record.doubles.find("y");
        if (xIt != record.doubles.end() && yIt != record.doubles.end()) {
          const glm::vec2 p = TransformPointXY(xIt->second, yIt->second);
          xIt->second = static_cast<double>(p.x);
          yIt->second = static_cast<double>(p.y);
        }
        if (auto zIt = record.doubles.find("z"); zIt != record.doubles.end()) {
          zIt->second = TransformZ(zIt->second);
        }
        if (auto thetaIt = record.doubles.find("theta"); thetaIt != record.doubles.end()) {
          thetaIt->second = TransformAngleRad(thetaIt->second);
        }
        if (auto yawIt = record.doubles.find("yaw"); yawIt != record.doubles.end()) {
          yawIt->second = TransformAngleRad(yawIt->second);
        }
      }
    }

    if (snapshot.pose.has_value()) {
      snapshot.pose = TransformPose(*snapshot.pose);
    }
    if (snapshot.activeGoal.has_value()) {
      snapshot.activeGoal = TransformPose(*snapshot.activeGoal);
    }
    if (snapshot.chosenCollect.has_value()) {
      snapshot.chosenCollect = TransformPose(*snapshot.chosenCollect);
    }
    if (snapshot.finalCollect.has_value()) {
      snapshot.finalCollect = TransformPose(*snapshot.finalCollect);
    }
  }

 private:
  Config cfg_;
  double cosRot_ = 1.0;
  double sinRot_ = 0.0;
};

}  // namespace repulsor3d
