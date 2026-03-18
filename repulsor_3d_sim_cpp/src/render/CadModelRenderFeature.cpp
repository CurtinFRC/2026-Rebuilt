#include "repulsor3d/render/CadModelRenderFeature.hpp"

#include <GL/glew.h>
#include <stb_image.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/trigonometric.hpp>
#include <glm/vec2.hpp>

#include "repulsor3d/Renderer.hpp"
#include "repulsor3d/render/cad/CadBroadphase.hpp"
#include "repulsor3d/render/cad/CadFrustum.hpp"
#include "repulsor3d/render/cad/CadPolicies.hpp"
#include "repulsor3d/render/cad/CadShaderSources.hpp"

namespace repulsor3d {
namespace {

#include "cad/feature/CadModelRenderFeature.Helpers.inc"

}  // namespace

#include "cad/feature/CadModelRenderFeature.Lifecycle.inc"
#include "cad/feature/CadModelRenderFeature.Render.inc"
#include "cad/feature/CadModelRenderFeature.Shader.inc"
#include "cad/feature/CadModelRenderFeature.Mesh.inc"
#include "cad/feature/CadModelRenderFeature.Assets.inc"

}  // namespace repulsor3d
