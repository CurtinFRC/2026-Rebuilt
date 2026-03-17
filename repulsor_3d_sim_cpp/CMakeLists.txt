cmake_minimum_required(VERSION 3.23)
project(repulsor_3d_sim_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(REPULSOR_SIM_FETCH_DEPS "Fetch dependencies with FetchContent" ON)
option(REPULSOR_SIM_USE_NTCORE "Enable NT4/NTCore datasource when available" ON)
option(REPULSOR_SIM_BUILD_TESTS "Build seam/unit tests" ON)

include(FetchContent)

if(REPULSOR_SIM_FETCH_DEPS)
  set(FETCHCONTENT_UPDATES_DISCONNECTED ON)

  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
  )
  FetchContent_MakeAvailable(glfw)

  FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.1
  )
  FetchContent_MakeAvailable(glm)

  FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
  )
  FetchContent_MakeAvailable(nlohmann_json)

  FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG master
  )
  FetchContent_MakeAvailable(stb)

  FetchContent_Declare(
    glew
    GIT_REPOSITORY https://github.com/Perlmint/glew-cmake.git
    GIT_TAG glew-cmake-2.2.0
  )
  FetchContent_MakeAvailable(glew)

  set(REPULSOR_GLEW_TARGET libglew_static)
  set(REPULSOR_STB_INCLUDE_DIR ${stb_SOURCE_DIR})
else()
  find_package(glfw3 REQUIRED)
  find_package(glm REQUIRED)
  find_package(nlohmann_json REQUIRED)
  find_package(GLEW REQUIRED)
  find_path(REPULSOR_STB_INCLUDE_DIR stb_image.h REQUIRED)
  set(REPULSOR_GLEW_TARGET GLEW::GLEW)
endif()

find_package(OpenGL REQUIRED)

set(REPULSOR_SIM_SOURCES
  src/main.cpp
  src/Config.cpp
  src/ConfigValidation.cpp
  src/Diagnostics.cpp
  src/TruthSocketReceiver.cpp
  src/SnapshotWorker.cpp
  src/NullDataSource.cpp
  src/Camera.cpp
  src/Renderer.cpp
  src/app/InputActionMap.cpp
  src/app/CameraControllers.cpp
  src/domain/SnapshotDomainAdapter.cpp
  src/sim/PoseSmoothing.cpp
  src/time/TimeSource.cpp
  src/time/FixedStepTicker.cpp
  src/render/RenderGraph.cpp
  src/render/RenderCommandBuffer.cpp
  src/render/RendererResources.cpp
  src/render/RendererText.cpp
  src/render/RenderFeature.cpp
  src/render/CadModelRenderFeature.cpp
  src/render/backend/RenderBackend.cpp
  src/render/assets/SceneAssetResolver.cpp
  src/render/geometry/GeometryProvider.cpp
  src/render/Season2026RebuiltModelBuilder.cpp
  src/render/SceneModelBuilderFactory.cpp
  src/App.cpp
)

add_executable(repulsor_3d_sim_cpp ${REPULSOR_SIM_SOURCES})

target_include_directories(repulsor_3d_sim_cpp
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${REPULSOR_STB_INCLUDE_DIR}
)

if(REPULSOR_SIM_FETCH_DEPS)
  target_include_directories(repulsor_3d_sim_cpp PRIVATE ${glm_SOURCE_DIR})
endif()

target_link_libraries(repulsor_3d_sim_cpp
  PRIVATE
    OpenGL::GL
    glfw
    ${REPULSOR_GLEW_TARGET}
    nlohmann_json::nlohmann_json
)

if(REPULSOR_SIM_USE_NTCORE)
  find_package(ntcore CONFIG QUIET)
  if(ntcore_FOUND)
    target_compile_definitions(repulsor_3d_sim_cpp PRIVATE REPULSOR_HAS_NTCORE=1)
    target_sources(repulsor_3d_sim_cpp PRIVATE src/NtDataSource.cpp)
    target_link_libraries(repulsor_3d_sim_cpp PRIVATE ntcore)
  else()
    message(STATUS "NTCore not found; building without NT4 datasource")
  endif()
endif()

if(WIN32)
  target_link_libraries(repulsor_3d_sim_cpp PRIVATE ws2_32)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../repulsor_3d_sim/field.png")
  add_custom_command(TARGET repulsor_3d_sim_cpp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${CMAKE_CURRENT_SOURCE_DIR}/../repulsor_3d_sim/field.png"
      "$<TARGET_FILE_DIR:repulsor_3d_sim_cpp>/field.png"
  )
endif()

if(REPULSOR_SIM_BUILD_TESTS)
  include(CTest)
  add_executable(repulsor_3d_sim_cpp_tests
    tests/PoseSmoothingTests.cpp
    src/sim/PoseSmoothing.cpp
    src/domain/SnapshotDomainAdapter.cpp
  )

  target_include_directories(repulsor_3d_sim_cpp_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  if(REPULSOR_SIM_FETCH_DEPS)
    target_include_directories(repulsor_3d_sim_cpp_tests PRIVATE ${glm_SOURCE_DIR})
  else()
    target_link_libraries(repulsor_3d_sim_cpp_tests PRIVATE glm::glm)
  endif()

  add_test(NAME repulsor_3d_sim_cpp_tests COMMAND repulsor_3d_sim_cpp_tests)
endif()
