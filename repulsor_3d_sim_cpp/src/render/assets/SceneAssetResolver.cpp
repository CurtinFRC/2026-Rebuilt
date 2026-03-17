#include "repulsor3d/render/assets/SceneAssetResolver.hpp"

#include <filesystem>
#include <vector>

namespace repulsor3d {

std::string DefaultSceneAssetResolver::ResolveFilePath(const std::string& rawPath) const {
  namespace fs = std::filesystem;
  if (rawPath.empty()) {
    return "";
  }

  const fs::path input(rawPath);
  if (input.is_absolute() && fs::exists(input) && fs::is_regular_file(input)) {
    return fs::absolute(input).string();
  }

  const fs::path cwd = fs::current_path();
  const std::vector<fs::path> candidates = {
      input,
      cwd / input,
      cwd / "assets" / input,
      cwd / "repulsor_3d_sim_cpp" / input,
      cwd / "repulsor_3d_sim_cpp" / "assets" / input,
      cwd / ".." / input,
      cwd / ".." / "repulsor_3d_sim_cpp" / input,
      cwd / ".." / "repulsor_3d_sim_cpp" / "assets" / input,
  };

  for (const auto& candidate : candidates) {
    if (fs::exists(candidate) && fs::is_regular_file(candidate)) {
      return fs::absolute(candidate).string();
    }
  }

  return "";
}

}  // namespace repulsor3d
