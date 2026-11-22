if(CMAKE_HOST_APPLE)
  cmake_minimum_required(VERSION 3.20)
  include(../../../cmake/MacLesson.cmake)
  return()
endif()

cmake_minimum_required(VERSION 3.1)
project(optical_flow LANGUAGES CXX)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui video)
add_executable(optical_flow src/optical_flow.cpp)
set_target_properties(optical_flow PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED ON)
target_include_directories(optical_flow PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../../common" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(optical_flow PRIVATE ${OpenCV_LIBS})
