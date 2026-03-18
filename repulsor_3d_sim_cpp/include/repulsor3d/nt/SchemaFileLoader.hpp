#pragma once

#if defined(REPULSOR_HAS_NTCORE)

#include <string>

#include "repulsor3d/nt/DefaultSchemas.hpp"

namespace repulsor3d::nt {

bool LoadSchemaSetFromFile(const std::string& path, NtSchemaSet& inOutSet, std::string* error = nullptr);

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)

