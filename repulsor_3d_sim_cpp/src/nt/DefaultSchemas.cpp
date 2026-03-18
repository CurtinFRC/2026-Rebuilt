#if defined(REPULSOR_HAS_NTCORE)

#include "repulsor3d/nt/DefaultSchemas.hpp"

#include "repulsor3d/nt/TopicPath.hpp"

namespace repulsor3d::nt {

EntityGroupSchema MakeFieldVisionObjectSchema(const ViewerConfig& cfg) {
  return EntityGroupSchema{
      .tablePrefix = TopicPrefix(cfg.fieldVisionPath),
      .idPrefix = "object_",
      .aliveField =
          {
              .key = "alive",
              .topicSuffix = "",
              .type = ::nt::NetworkTableType::kBoolean,
              .defaultDouble = 0.0,
              .defaultBoolean = false,
              .defaultString = "",
          },
      .fields = {
          {
              .key = "type",
              .topicSuffix = "type",
              .type = ::nt::NetworkTableType::kString,
              .defaultDouble = 0.0,
              .defaultBoolean = false,
              .defaultString = "",
          },
          {.key = "x", .topicSuffix = "x"},
          {.key = "y", .topicSuffix = "y"},
          {.key = "z", .topicSuffix = "z"},
          {.key = "roll", .topicSuffix = "roll"},
          {.key = "pitch", .topicSuffix = "pitch"},
          {.key = "yaw", .topicSuffix = "yaw"},
      },
      .autoCaptureAdditionalScalars = true,
      .extraKeyPrefix = "extra.",
  };
}

EntityGroupSchema MakeRepulsorObstacleSchema(const ViewerConfig& cfg) {
  return EntityGroupSchema{
      .tablePrefix = TopicPrefix(cfg.repulsorVisionPath),
      .idPrefix = "obs_",
      .aliveField =
          {
              .key = "alive",
              .topicSuffix = "",
              .type = ::nt::NetworkTableType::kBoolean,
              .defaultDouble = 0.0,
              .defaultBoolean = false,
              .defaultString = "",
          },
      .fields = {
          {
              .key = "kind",
              .topicSuffix = "kind",
              .type = ::nt::NetworkTableType::kString,
              .defaultDouble = 0.0,
              .defaultBoolean = false,
              .defaultString = "",
          },
          {.key = "x", .topicSuffix = "x"},
          {.key = "y", .topicSuffix = "y"},
          {.key = "size_x", .topicSuffix = "size_x"},
          {.key = "size_y", .topicSuffix = "size_y"},
      },
      .autoCaptureAdditionalScalars = true,
      .extraKeyPrefix = "extra.",
  };
}

EntityGroupSchema MakeCameraSchema(const ViewerConfig& cfg) {
  return EntityGroupSchema{
      .tablePrefix = TopicPrefix(cfg.fieldVisionPath),
      .idPrefix = "camera_",
      .aliveField =
          {
              .key = "alive",
              .topicSuffix = "",
              .type = ::nt::NetworkTableType::kBoolean,
              .defaultDouble = 0.0,
              .defaultBoolean = false,
              .defaultString = "",
          },
      .fields = {
          {.key = "x", .topicSuffix = "x"},
          {.key = "y", .topicSuffix = "y"},
          {.key = "z", .topicSuffix = "z"},
          {.key = "yaw_deg", .topicSuffix = "yaw_deg"},
          {.key = "pitch_deg", .topicSuffix = "pitch_deg"},
          {.key = "roll_deg", .topicSuffix = "roll_deg"},
          {.key = "hfov_deg", .topicSuffix = "hfov_deg"},
          {.key = "vfov_deg", .topicSuffix = "vfov_deg"},
          {.key = "max_range", .topicSuffix = "max_range"},
      },
      .autoCaptureAdditionalScalars = true,
      .extraKeyPrefix = "extra.",
  };
}

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)

