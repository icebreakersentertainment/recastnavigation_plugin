cmake_minimum_required(VERSION 3.1.0)

project(recastnavigation_plugin)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(RECASTNAVIGATION_PLUGIN_COMPILER_FLAGS "")
set(RECASTNAVIGATION_PLUGIN_DEFINITIONS "")
if(MSVC)
  list(APPEND RECASTNAVIGATION_PLUGIN_COMPILER_FLAGS /EHsc /MP)
  if(CMAKE_BUILD_TYPE MATCHES DEBUG)
    list(APPEND RECASTNAVIGATION_PLUGIN_COMPILER_FLAGS /EHsc /MP)
  endif()
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug OR CMAKE_BUILD_TYPE MATCHES RelWithDebInfo OR ICEENGINE_ENABLE_DEBUG_LOGGING)
  list(APPEND RECASTNAVIGATION_PLUGIN_DEFINITIONS -DICEENGINE_ENABLE_DEBUG_LOGGING)
endif()

if(ICEENGINE_ENABLE_TRACE_LOGGING)
  list(APPEND RECASTNAVIGATION_PLUGIN_DEFINITIONS -DICEENGINE_ENABLE_TRACE_LOGGING)
endif()

find_package(glm REQUIRED)
find_package(Boost REQUIRED COMPONENTS stacktrace)

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
list(APPEND SOURCES "${ICEENGINE_BASE_DIR}/src/exceptions/Exception.cpp")
list(APPEND SOURCES "${ICEENGINE_BASE_DIR}/src/exceptions/Stacktrace.cpp")

add_library(recastnavigation_plugin SHARED ${SOURCES})

# Headers
target_include_directories(recastnavigation_plugin PRIVATE include)

target_include_directories(recastnavigation_plugin PRIVATE ./recastnavigation/recastnavigation/Detour/Include)
target_include_directories(recastnavigation_plugin PRIVATE ./recastnavigation/recastnavigation/DetourCrowd/Include)
target_include_directories(recastnavigation_plugin PRIVATE ./recastnavigation/recastnavigation/DetourTileCache/Include)
target_include_directories(recastnavigation_plugin PRIVATE ./recastnavigation/recastnavigation/DebugUtils/Include)
target_include_directories(recastnavigation_plugin PRIVATE ./recastnavigation/recastnavigation/Recast/Include)
target_include_directories(recastnavigation_plugin PRIVATE ./recastnavigation/recastnavigation/RecastDemo/Include)
target_include_directories(recastnavigation_plugin PRIVATE ./recastnavigation/recastnavigation/RecastDemo/Contrib/fastlz)

target_include_directories(recastnavigation_plugin PRIVATE ${ICEENGINE_INCLUDE_DIRS})

target_compile_definitions(recastnavigation_plugin PRIVATE ${RECASTNAVIGATION_PLUGIN_DEFINITIONS})
target_compile_options(recastnavigation_plugin PRIVATE ${RECASTNAVIGATION_PLUGIN_COMPILER_FLAGS})

target_link_libraries(recastnavigation_plugin PRIVATE glm::glm)
target_link_libraries(recastnavigation_plugin PRIVATE Boost::stacktrace)