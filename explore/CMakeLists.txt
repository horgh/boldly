cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Release)
set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()

# Library
rosbuild_add_library(explore_lib
src/explore_frontier.cpp
src/loop_closure.cpp
src/notifier.cpp
src/explore.cpp)
set_target_properties(explore_lib
                      PROPERTIES OUTPUT_NAME "explore")
rosbuild_link_boost(explore_lib thread)

# Explore
rosbuild_add_executable(explore src/explore.cpp)
target_link_libraries(explore explore_lib)
#target_link_libraries(explore skeleplanner)
target_link_libraries(explore topomap)

# Find max
rosbuild_add_executable(find_max src/find_max.cpp)
target_link_libraries(find_max explore_lib)
target_link_libraries(find_max topomap)
