# Use the g2o version whose solver API is used in the lectures.
function(add_lecture_g2o)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.10)
  foreach(package IN ITEMS QGLViewer Cholmod CSparse)
    set(CMAKE_DISABLE_FIND_PACKAGE_${package} TRUE)
  endforeach()
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  foreach(option IN ITEMS G2O_BUILD_APPS G2O_BUILD_EXAMPLES G2O_USE_OPENGL G2O_USE_CHOLMOD G2O_USE_CSPARSE)
    set(${option} OFF CACHE BOOL "" FORCE)
  endforeach()
  foreach(extension IN ITEMS SSE2 SSE3 SSE4_1 SSE4_2 SSE4_A)
    set(DISABLE_${extension} ON CACHE BOOL "" FORCE)
  endforeach()
  set(g2o_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib" CACHE PATH "")
  set(g2o_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin" CACHE PATH "")
  add_subdirectory(3rdparty/g2o vendor/g2o EXCLUDE_FROM_ALL)
  target_include_directories(core PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rdparty/g2o>"
    "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/vendor/g2o>")
endfunction()
add_lecture_g2o()

# DBoW3's public headers use the installed DBoW3/ include layout.
set(dbow_src "${PROJECT_SOURCE_DIR}/3rdparty/DBoW3/src")
file(GLOB dbow_headers CONFIGURE_DEPENDS "${dbow_src}/*.h")
foreach(header IN LISTS dbow_headers)
  get_filename_component(name "${header}" NAME)
  configure_file("${header}" "${CMAKE_BINARY_DIR}/include/DBoW3/${name}" COPYONLY)
endforeach()
add_library(lecture_dbow3 STATIC
  ${dbow_src}/BowVector.cpp ${dbow_src}/Database.cpp ${dbow_src}/DescManip.cpp
  ${dbow_src}/FeatureVector.cpp ${dbow_src}/QueryResults.cpp ${dbow_src}/ScoringObject.cpp
  ${dbow_src}/Vocabulary.cpp ${dbow_src}/quicklz.c)
target_include_directories(lecture_dbow3 PUBLIC "${CMAKE_BINARY_DIR}/include")
target_link_libraries(lecture_dbow3 PUBLIC ${OpenCV_LIBS})

# These are the OpenGV components used by the triangulation exercise.
set(OPENGV_SOURCE_DIR "${PROJECT_SOURCE_DIR}/build/opengv-src" CACHE PATH "OpenGV checkout")
add_library(lecture_opengv STATIC
  ${OPENGV_SOURCE_DIR}/src/triangulation/methods.cpp
  ${OPENGV_SOURCE_DIR}/src/relative_pose/CentralRelativeAdapter.cpp
  ${OPENGV_SOURCE_DIR}/src/math/cayley.cpp)
target_include_directories(lecture_opengv PUBLIC "${OPENGV_SOURCE_DIR}/include")
target_link_libraries(lecture_opengv PUBLIC Eigen3::Eigen)
