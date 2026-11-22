if(CMAKE_HOST_APPLE)
  cmake_minimum_required(VERSION 3.20)
  include(../../../cmake/MacLesson.cmake)
  return()
endif()

cmake_minimum_required(VERSION 2.8)
project(vo1)

set(CMAKE_BUILD_TYPE "Release")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS}")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)
find_package(G2O REQUIRED)

include_directories(
        "${CMAKE_CURRENT_SOURCE_DIR}/../../../common"
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        "/usr/include/eigen3/"
)

add_executable(pose_estimation_3d2d src/pose_estimation_3d2d.cpp)
target_include_directories(pose_estimation_3d2d PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdparty/Sophus")
target_link_libraries(pose_estimation_3d2d
        g2o_core g2o_stuff
        ${OpenCV_LIBS})
