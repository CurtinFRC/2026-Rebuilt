#pragma once

#if defined(REPULSOR_HAS_NTCORE)

#include <string>
#include <vector>

#include "repulsor3d/Config.hpp"
#include "repulsor3d/nt/TopicSchema.hpp"

namespace repulsor3d::nt {

struct NtSchemaSet {
  struct DynamicChannelSchema {
    std::string channel;
    EntityGroupSchema schema;
  };

  EntityGroupSchema fieldVision;
  EntityGroupSchema repulsor;
  EntityGroupSchema cameras;
  std::vector<DynamicChannelSchema> dynamicChannels;
};

EntityGroupSchema MakeFieldVisionObjectSchema(const ViewerConfig& cfg);
EntityGroupSchema MakeRepulsorObstacleSchema(const ViewerConfig& cfg);
EntityGroupSchema MakeCameraSchema(const ViewerConfig& cfg);
NtSchemaSet MakeDefaultSchemaSet(const ViewerConfig& cfg);

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)
