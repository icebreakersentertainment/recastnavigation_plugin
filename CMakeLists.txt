cmake_minimum_required(VERSION 3.1.0)

project(recastnavigation_plugin)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(RECASTNAVIGATION_PLUGIN_COMPILER_FLAGS "")
set(RECASTNAVIGATION_PLUGIN_DEFINITIONS "")
if(MSVC)
  set(RECASTNAVIGATION_PLUGIN_COMPILER_FLAGS "/EHsc /MP")
  if(CMAKE_BUILD_TYPE MATCHES DEBUG)
    set(RECASTNAVIGATION_PLUGIN_COMPILER_FLAGS "/EHsc /MP")
  endif()
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug OR CMAKE_BUILD_TYPE MATCHES RelWithDebInfo OR ICEENGINE_ENABLE_DEBUG_LOGGING)
  list(APPEND SQLITECPP_PLUGIN_DEFINITIONS -DICEENGINE_ENABLE_DEBUG_LOGGING)
endif()

if(ICEENGINE_ENABLE_TRACE_LOGGING)
  list(APPEND SQLITECPP_PLUGIN_DEFINITIONS -DICEENGINE_ENABLE_TRACE_LOGGING)
endif()

include_directories(./recastnavigation/recastnavigation/Detour/Include)
include_directories(./recastnavigation/recastnavigation/DetourCrowd/Include)
include_directories(./recastnavigation/recastnavigation/DetourTileCache/Include)
include_directories(./recastnavigation/recastnavigation/DebugUtils/Include)
include_directories(./recastnavigation/recastnavigation/Recast/Include)
include_directories(./recastnavigation/recastnavigation/RecastDemo/Include)
include_directories(./recastnavigation/recastnavigation/RecastDemo/Contrib/fastlz)

# Headers
include_directories(include)

# Source
file(
  GLOB_RECURSE
  SOURCES
  "src/*.cpp"
  "recastnavigation/recastnavigation/RecastDemo/Contrib/fastlz/*.c"
  "recastnavigation/recastnavigation/Detour/Source/*.cpp"
  "recastnavigation/recastnavigation/DetourTileCache/Source/*.cpp"
  "recastnavigation/recastnavigation/DetourCrowd/Source/*.cpp"
  "recastnavigation/recastnavigation/DebugUtils/Source/*.cpp"
  "recastnavigation/recastnavigation/Recast/Source/*.cpp"
  "recastnavigation/recastnavigation/RecastDemo/Source/ChunkyTriMesh.cpp"
)

add_library(recastnavigation_plugin SHARED ${SOURCES})

target_include_directories(recastnavigation_plugin PRIVATE ${ICEENGINE_INCLUDE_DIRS})
target_include_directories(recastnavigation_plugin PRIVATE ${BOOST_INCLUDE_DIRS})
target_include_directories(recastnavigation_plugin PRIVATE ${GLM_INCLUDE_DIRS})

target_compile_definitions(recastnavigation_plugin PRIVATE ${SQLITECPP_PLUGIN_DEFINITIONS})
target_compile_options(recastnavigation_plugin PRIVATE ${RECASTNAVIGATION_PLUGIN_COMPILER_FLAGS})