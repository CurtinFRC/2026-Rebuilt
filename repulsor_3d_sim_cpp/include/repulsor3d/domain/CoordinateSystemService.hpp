#pragma once

#include <cstdint>
#include <optional>
#include <unordered_map>

#include <glm/vec2.hpp>

#include "repulsor3d/Model.hpp"
#include "repulsor3d/domain/CoordinateFrameMapper.hpp"

namespace repulsor3d {

enum class CoordinateFrameId : std::uint8_t {
  Incoming = 0,
  Field = 1,
  Cad = 2,
  Render = 3,
  Screen = 4,
};

class CoordinateSystemService {
 public:
  void SetTransform(CoordinateFrameId from, CoordinateFrameId to, CoordinateFrameMapper::Config config);
  bool HasTransform(CoordinateFrameId from, CoordinateFrameId to) const;

  glm::vec2 TransformPointXY(CoordinateFrameId from, CoordinateFrameId to, double x, double y) const;
  double TransformZ(CoordinateFrameId from, CoordinateFrameId to, double z) const;
  Pose2D TransformPose(CoordinateFrameId from, CoordinateFrameId to, const Pose2D& pose) const;
  void TransformSnapshot(CoordinateFrameId from, CoordinateFrameId to, WorldSnapshot& snapshot) const;

 private:
  struct TransformKey {
    CoordinateFrameId from = CoordinateFrameId::Incoming;
    CoordinateFrameId to = CoordinateFrameId::Field;

    bool operator==(const TransformKey& other) const {
      return from == other.from && to == other.to;
    }
  };

  struct TransformKeyHash {
    std::size_t operator()(const TransformKey& key) const {
      return (static_cast<std::size_t>(key.from) << 8U) ^ static_cast<std::size_t>(key.to);
    }
  };

  const CoordinateFrameMapper* FindMapper(CoordinateFrameId from, CoordinateFrameId to) const;

  std::unordered_map<TransformKey, CoordinateFrameMapper, TransformKeyHash> mappers_;
};

}  // namespace repulsor3d
