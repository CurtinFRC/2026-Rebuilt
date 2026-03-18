#pragma once

#if defined(REPULSOR_HAS_NTCORE)

#include "repulsor3d/Config.hpp"
#include "repulsor3d/nt/TopicSchema.hpp"

namespace repulsor3d::nt {

EntityGroupSchema MakeFieldVisionObjectSchema(const ViewerConfig& cfg);
EntityGroupSchema MakeRepulsorObstacleSchema(const ViewerConfig& cfg);
EntityGroupSchema MakeCameraSchema(const ViewerConfig& cfg);

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)

