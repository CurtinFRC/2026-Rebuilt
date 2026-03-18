cmake_minimum_required(VERSION 3.23)
project(repulsor_3d_sim_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(REPULSOR_SIM_FETCH_DEPS "Fetch dependencies with FetchContent" ON)
option(REPULSOR_SIM_USE_NTCORE "Enable NT4/NTCore datasource when available" ON)
option(REPULSOR_SIM_BUILD_TESTS "Build seam/unit tests" ON)
option(REPULSOR_SIM_BUILD_PLUGIN_SDK "Build sample season plugin SDK target + ABI checks" ON)
set(REPULSOR_SIM_NTCORE_ROOT "C:/Users/paulh/allwpilib" CACHE PATH "Root directory of a local allwpilib checkout/build for ntcore")
set(REPULSOR_SIM_NTCORE_HINTS "" CACHE STRING "Optional semicolon-separated extra hint paths for ntcore package discovery")
set(REPULSOR_SIM_NTCORE_SOURCES
  src/NtDataSource.cpp
  src/nt/DefaultSchemas.cpp
  src/nt/DomainMappers.cpp
  src/nt/TopicPath.cpp
  src/nt/SubscriberCollection.cpp
  src/nt/DynamicEntityRegistry.cpp
)

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
  src/sim/SimWorld.cpp
  src/time/TimeSource.cpp
  src/time/FixedStepTicker.cpp
  src/render/RenderGraph.cpp
  src/render/RenderCommandBuffer.cpp
  src/render/RenderWorldAdapter.cpp
  src/render/SceneDescriptor.cpp
  src/render/ecs/EntityRegistry.cpp
  src/render/ecs/Systems.cpp
  src/render/RendererResources.cpp
  src/render/RendererText.cpp
  src/render/RenderFeature.cpp
  src/render/CadModelRenderFeature.cpp
  src/render/backend/RenderBackend.cpp
  src/render/assets/CadMeshImportHelpers.cpp
  src/render/assets/CadMeshStlImporter.cpp
  src/render/assets/CadMeshObjImporter.cpp
  src/render/assets/CadMeshGltfImporter.cpp
  src/render/assets/CadMeshImporterFactory.cpp
  src/render/assets/CadMeshCache.cpp
  src/render/assets/CadMeshAssetPipeline.cpp
  src/render/assets/SceneAssetResolver.cpp
  src/render/geometry/GeometryProvider.cpp
  src/render/Season2026RebuiltModelBuilder.cpp
  src/render/SceneModelBuilderFactory.cpp
  src/modules/SeasonModule.cpp
  src/App.cpp
)

add_executable(repulsor_3d_sim_cpp ${REPULSOR_SIM_SOURCES})
if(MSVC)
  target_compile_options(repulsor_3d_sim_cpp PRIVATE /utf-8)
endif()

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

if(REPULSOR_SIM_BUILD_PLUGIN_SDK)
  add_library(repulsor3d_generic_season_plugin SHARED
    plugin_sdk/GenericSeasonPluginTemplate.cpp
  )
  target_include_directories(repulsor3d_generic_season_plugin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  if(REPULSOR_SIM_FETCH_DEPS)
    target_include_directories(repulsor3d_generic_season_plugin PRIVATE ${glm_SOURCE_DIR})
  else()
    target_link_libraries(repulsor3d_generic_season_plugin PRIVATE glm::glm)
  endif()
  if(MSVC)
    target_compile_options(repulsor3d_generic_season_plugin PRIVATE /utf-8)
  endif()
endif()

if(REPULSOR_SIM_USE_NTCORE)
  set(_repulsor_ntcore_search_paths
    "${REPULSOR_SIM_NTCORE_ROOT}"
    "${REPULSOR_SIM_NTCORE_ROOT}/build"
    "${REPULSOR_SIM_NTCORE_ROOT}/build/ntcore"
    "${REPULSOR_SIM_NTCORE_ROOT}/install"
    "${REPULSOR_SIM_NTCORE_ROOT}/build/install"
    ${REPULSOR_SIM_NTCORE_HINTS}
  )

  find_package(ntcore CONFIG QUIET
    PATHS ${_repulsor_ntcore_search_paths}
    PATH_SUFFIXES lib/cmake/ntcore ntcore cmake
    NO_DEFAULT_PATH
  )

  if(ntcore_FOUND)
    message(STATUS "NTCore found locally: ${ntcore_DIR}")
    target_compile_definitions(repulsor_3d_sim_cpp PRIVATE REPULSOR_HAS_NTCORE=1)
    target_sources(repulsor_3d_sim_cpp PRIVATE ${REPULSOR_SIM_NTCORE_SOURCES})
    target_link_libraries(repulsor_3d_sim_cpp PRIVATE ntcore)
  else()
    set(_repulsor_ntcore_include_candidates
      "${REPULSOR_SIM_NTCORE_ROOT}/ntcore/src/main/native/include"
      ${REPULSOR_SIM_NTCORE_HINTS}
    )
    set(_repulsor_ntcore_generated_include_candidates
      "${REPULSOR_SIM_NTCORE_ROOT}/ntcore/src/generated/main/native/include"
      "${REPULSOR_SIM_NTCORE_ROOT}/ntcore/src/generate/main/native/include"
      ${REPULSOR_SIM_NTCORE_HINTS}
    )
    set(_repulsor_wpinet_include_candidates
      "${REPULSOR_SIM_NTCORE_ROOT}/wpinet/src/main/native/include"
      ${REPULSOR_SIM_NTCORE_HINTS}
    )
    set(_repulsor_wpiutil_include_candidates
      "${REPULSOR_SIM_NTCORE_ROOT}/wpiutil/src/main/native/include"
      ${REPULSOR_SIM_NTCORE_HINTS}
    )
    file(GLOB _repulsor_wpiutil_thirdparty_include_dirs LIST_DIRECTORIES true
      "${REPULSOR_SIM_NTCORE_ROOT}/wpiutil/src/main/native/thirdparty/*/include"
    )
    file(GLOB _repulsor_wpinet_thirdparty_include_dirs LIST_DIRECTORIES true
      "${REPULSOR_SIM_NTCORE_ROOT}/wpinet/src/main/native/thirdparty/*/include"
    )

    set(_repulsor_ntcore_lib_candidates
      "${REPULSOR_SIM_NTCORE_ROOT}/ntcore/build/libs/ntcore/shared/windowsx86-64/release"
      "${REPULSOR_SIM_NTCORE_ROOT}/ntcore/build/install/ntcoreDev/windowsx86-64/lib"
      ${REPULSOR_SIM_NTCORE_HINTS}
    )
    set(_repulsor_wpinet_lib_candidates
      "${REPULSOR_SIM_NTCORE_ROOT}/wpinet/build/libs/wpinet/shared/windowsx86-64/release"
      "${REPULSOR_SIM_NTCORE_ROOT}/ntcore/build/install/ntcoreDev/windowsx86-64/lib"
      ${REPULSOR_SIM_NTCORE_HINTS}
    )
    set(_repulsor_wpiutil_lib_candidates
      "${REPULSOR_SIM_NTCORE_ROOT}/wpiutil/build/libs/wpiutil/shared/windowsx86-64/release"
      "${REPULSOR_SIM_NTCORE_ROOT}/ntcore/build/install/ntcoreDev/windowsx86-64/lib"
      ${REPULSOR_SIM_NTCORE_HINTS}
    )

    find_path(REPULSOR_RAW_NTCORE_INCLUDE_DIR ntcore.h PATHS ${_repulsor_ntcore_include_candidates})
    find_path(
      REPULSOR_RAW_NTCORE_GENERATED_INCLUDE_DIR
      networktables/BooleanTopic.h
      PATHS ${_repulsor_ntcore_generated_include_candidates}
    )
    find_path(REPULSOR_RAW_WPINET_INCLUDE_DIR wpinet/WebServer.h PATHS ${_repulsor_wpinet_include_candidates})
    find_path(REPULSOR_RAW_WPIUTIL_INCLUDE_DIR wpi/DataLog.h PATHS ${_repulsor_wpiutil_include_candidates})

    find_library(REPULSOR_RAW_NTCORE_LIB ntcore PATHS ${_repulsor_ntcore_lib_candidates})
    find_library(REPULSOR_RAW_WPINET_LIB wpinet PATHS ${_repulsor_wpinet_lib_candidates})
    find_library(REPULSOR_RAW_WPIUTIL_LIB wpiutil PATHS ${_repulsor_wpiutil_lib_candidates})

    if(
      REPULSOR_RAW_NTCORE_INCLUDE_DIR
      AND REPULSOR_RAW_NTCORE_GENERATED_INCLUDE_DIR
      AND REPULSOR_RAW_WPINET_INCLUDE_DIR
      AND REPULSOR_RAW_WPIUTIL_INCLUDE_DIR
      AND REPULSOR_RAW_NTCORE_LIB
      AND REPULSOR_RAW_WPINET_LIB
      AND REPULSOR_RAW_WPIUTIL_LIB
    )
      message(STATUS "NTCore found via raw allwpilib layout under '${REPULSOR_SIM_NTCORE_ROOT}'")
      target_compile_definitions(repulsor_3d_sim_cpp PRIVATE REPULSOR_HAS_NTCORE=1)
      target_sources(repulsor_3d_sim_cpp PRIVATE ${REPULSOR_SIM_NTCORE_SOURCES})
      target_include_directories(
        repulsor_3d_sim_cpp PRIVATE
          ${REPULSOR_RAW_NTCORE_INCLUDE_DIR}
          ${REPULSOR_RAW_NTCORE_GENERATED_INCLUDE_DIR}
          ${REPULSOR_RAW_WPINET_INCLUDE_DIR}
          ${REPULSOR_RAW_WPIUTIL_INCLUDE_DIR}
          ${_repulsor_wpiutil_thirdparty_include_dirs}
          ${_repulsor_wpinet_thirdparty_include_dirs}
      )
      target_link_libraries(
        repulsor_3d_sim_cpp PRIVATE
          ${REPULSOR_RAW_NTCORE_LIB}
          ${REPULSOR_RAW_WPINET_LIB}
          ${REPULSOR_RAW_WPIUTIL_LIB}
      )

      get_filename_component(_repulsor_ntcore_lib_dir "${REPULSOR_RAW_NTCORE_LIB}" DIRECTORY)
      get_filename_component(_repulsor_wpinet_lib_dir "${REPULSOR_RAW_WPINET_LIB}" DIRECTORY)
      get_filename_component(_repulsor_wpiutil_lib_dir "${REPULSOR_RAW_WPIUTIL_LIB}" DIRECTORY)

      set(_repulsor_runtime_dlls
        "${_repulsor_ntcore_lib_dir}/ntcore.dll"
        "${_repulsor_wpinet_lib_dir}/wpinet.dll"
        "${_repulsor_wpiutil_lib_dir}/wpiutil.dll"
      )
      foreach(_repulsor_runtime_dll IN LISTS _repulsor_runtime_dlls)
        if(EXISTS "${_repulsor_runtime_dll}")
          add_custom_command(TARGET repulsor_3d_sim_cpp POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "${_repulsor_runtime_dll}"
              "$<TARGET_FILE_DIR:repulsor_3d_sim_cpp>"
          )
        endif()
      endforeach()
    else()
      message(STATUS "NTCore not found under REPULSOR_SIM_NTCORE_ROOT='${REPULSOR_SIM_NTCORE_ROOT}'; building without NT4 datasource")
      message(STATUS "Set REPULSOR_SIM_NTCORE_ROOT or REPULSOR_SIM_NTCORE_HINTS to your local allwpilib build/install path")
    endif()
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

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/assets/scenes")
  add_custom_command(TARGET repulsor_3d_sim_cpp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
      "${CMAKE_CURRENT_SOURCE_DIR}/assets/scenes"
      "$<TARGET_FILE_DIR:repulsor_3d_sim_cpp>/assets/scenes"
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/assets/field_2026rebuilt.gltf")
  add_custom_command(TARGET repulsor_3d_sim_cpp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${CMAKE_CURRENT_SOURCE_DIR}/assets/field_2026rebuilt.gltf"
      "$<TARGET_FILE_DIR:repulsor_3d_sim_cpp>/field_2026rebuilt.gltf"
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

  add_executable(repulsor_3d_sim_cpp_module_tests
    tests/ModuleAbstractionTests.cpp
    src/sim/SimWorld.cpp
    src/render/SceneDescriptor.cpp
    src/render/RenderWorldAdapter.cpp
    src/render/Season2026RebuiltModelBuilder.cpp
    src/render/SceneModelBuilderFactory.cpp
    src/modules/SeasonModule.cpp
  )

  target_include_directories(repulsor_3d_sim_cpp_module_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(repulsor_3d_sim_cpp_module_tests PRIVATE nlohmann_json::nlohmann_json)
  if(REPULSOR_SIM_FETCH_DEPS)
    target_include_directories(repulsor_3d_sim_cpp_module_tests PRIVATE ${glm_SOURCE_DIR})
  else()
    target_link_libraries(repulsor_3d_sim_cpp_module_tests PRIVATE glm::glm)
  endif()

  add_test(NAME repulsor_3d_sim_cpp_module_tests COMMAND repulsor_3d_sim_cpp_module_tests)

  if(REPULSOR_SIM_BUILD_PLUGIN_SDK)
    add_executable(repulsor_3d_sim_cpp_plugin_abi_tests
      tests/PluginAbiSmokeTests.cpp
      src/sim/SimWorld.cpp
      src/render/SceneDescriptor.cpp
      src/render/RenderWorldAdapter.cpp
      src/render/Season2026RebuiltModelBuilder.cpp
      src/render/SceneModelBuilderFactory.cpp
      src/modules/SeasonModule.cpp
    )

    target_include_directories(repulsor_3d_sim_cpp_plugin_abi_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(repulsor_3d_sim_cpp_plugin_abi_tests PRIVATE nlohmann_json::nlohmann_json)
    if(REPULSOR_SIM_FETCH_DEPS)
      target_include_directories(repulsor_3d_sim_cpp_plugin_abi_tests PRIVATE ${glm_SOURCE_DIR})
    else()
      target_link_libraries(repulsor_3d_sim_cpp_plugin_abi_tests PRIVATE glm::glm)
    endif()

    if(MSVC)
      target_compile_options(repulsor_3d_sim_cpp_plugin_abi_tests PRIVATE /utf-8)
    endif()

    add_dependencies(repulsor_3d_sim_cpp_plugin_abi_tests repulsor3d_generic_season_plugin)
    add_test(
      NAME repulsor_3d_sim_cpp_plugin_abi_tests
      COMMAND
        ${CMAKE_COMMAND} -E env
        REPULSOR_TEST_PLUGIN_PATH=$<TARGET_FILE:repulsor3d_generic_season_plugin>
        $<TARGET_FILE:repulsor_3d_sim_cpp_plugin_abi_tests>
    )

    add_custom_target(repulsor_plugin_abi_check
      COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIG> --output-on-failure -R repulsor_3d_sim_cpp_plugin_abi_tests
      DEPENDS repulsor_3d_sim_cpp_plugin_abi_tests repulsor3d_generic_season_plugin
    )
  endif()
endif()
