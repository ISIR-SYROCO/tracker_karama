cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include(FindPkgConfig)
pkg_check_modules(OPENNI libopenni)

include_directories(${OPENNI_INCLUDEDIR} /usr/include/nite)

rosbuild_add_executable(tracker src/tracker.cpp)
target_link_libraries(tracker ${OPENNI_LIBRARIES})
set(CMAKE_CXX_FLAGS "-ggdb")
