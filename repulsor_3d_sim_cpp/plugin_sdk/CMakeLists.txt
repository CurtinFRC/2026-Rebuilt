cmake_minimum_required(VERSION 3.23)
project(repulsor3d_plugin_sdk_template LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(glm QUIET)

add_library(repulsor3d_generic_season_plugin SHARED GenericSeasonPluginTemplate.cpp)
target_include_directories(
  repulsor3d_generic_season_plugin
  PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../include
)

add_library(repulsor3d_generic_datasource_plugin SHARED GenericDataSourcePluginTemplate.cpp)
target_include_directories(
  repulsor3d_generic_datasource_plugin
  PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../include
)

add_library(repulsor3d_generic_render_feature_plugin SHARED GenericRenderFeaturePluginTemplate.cpp)
target_include_directories(
  repulsor3d_generic_render_feature_plugin
  PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../include
)
if(glm_FOUND)
  target_link_libraries(repulsor3d_generic_render_feature_plugin PRIVATE glm::glm)
endif()

if(MSVC)
  target_compile_options(repulsor3d_generic_season_plugin PRIVATE /utf-8)
  target_compile_options(repulsor3d_generic_datasource_plugin PRIVATE /utf-8)
  target_compile_options(repulsor3d_generic_render_feature_plugin PRIVATE /utf-8)
endif()
