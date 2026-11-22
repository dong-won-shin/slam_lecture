cmake_minimum_required(VERSION 3.20)
project(slam_lecture LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs features2d calib3d highgui)
find_package(Ceres 2.2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS io filters features segmentation registration visualization)

include(cmake/LectureDependencies.cmake)

function(lecture_example name lesson)
  list(TRANSFORM ARGN PREPEND "${lesson}/src/")
  add_executable(${name} ${ARGN})
  set(lesson_dir "${PROJECT_SOURCE_DIR}/${lesson}/build")
  set_target_properties(${name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${lesson_dir}")

  # Keep existing commands that launch examples from the central bin directory.
  set(link "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${name}")
  add_custom_command(TARGET ${name} POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
    COMMAND "${CMAKE_COMMAND}" -E create_symlink "$<TARGET_FILE:${name}>" "${link}"
    BYPRODUCTS "${link}" VERBATIM)

  if(CMAKE_GENERATOR STREQUAL "Ninja")
    string(MAKE_C_IDENTIFIER "lesson_${lesson}" lesson_target)
    if(NOT TARGET ${lesson_target})
      add_custom_target(${lesson_target})
      if(NOT EXISTS "${lesson_dir}/CMakeCache.txt")
        set(shared_build_dir "${CMAKE_BINARY_DIR}")
        configure_file(cmake/lesson.ninja.in "${lesson_dir}/build.ninja" @ONLY)
      endif()
    endif()
    add_dependencies(${lesson_target} ${name})
  endif()
endfunction()

lecture_example(3d_point_camera_projection 3DPointCameraProjection main.cpp utils.cpp)
target_link_libraries(3d_point_camera_projection PRIVATE ${OpenCV_LIBS})

foreach(name IN ITEMS pass_through_filter voxel_grid_filter statistical_outlier_filter normal_estimation clustering)
  lecture_example(${name} 3DPointCloudProcessing ${name}.cpp)
  target_link_libraries(${name} PRIVATE pcl_io pcl_filters)
endforeach()
target_link_libraries(normal_estimation PRIVATE pcl_features pcl_visualization)
target_link_libraries(clustering PRIVATE pcl_segmentation)
lecture_example(iterative_form_registration 3DPointCloudRegistration iterative_form_registration.cpp)
target_link_libraries(iterative_form_registration PRIVATE pcl_io pcl_registration)
lecture_example(closed_form_registration 3DPointCloudRegistration closed_form_registration.cpp)
target_link_libraries(closed_form_registration PRIVATE Eigen3::Eigen ${OpenCV_LIBS})

lecture_example(bundle_adjustment_ceres BundleAdjustment bundle_adjustment_ceres.cpp common.cpp)
target_include_directories(bundle_adjustment_ceres PRIVATE BundleAdjustment/include)
target_link_libraries(bundle_adjustment_ceres PRIVATE Ceres::ceres)

lecture_example(pose_estimation_2d2d "Fundamental&EssentialMatrix" pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d PRIVATE ${OpenCV_LIBS})
lecture_example(pose_graph_g2o_SE3 GraphOptimization pose_graph_g2o_SE3.cpp)
target_link_libraries(pose_graph_g2o_SE3 PRIVATE types_slam3d)
lecture_example(kalman-test KalmanFilter kalman.cpp kalman-test.cpp)
target_link_libraries(kalman-test PRIVATE Eigen3::Eigen)

foreach(name IN ITEMS orb_cv orb_self)
  lecture_example(${name} "Keypoint&Descriptor" ${name}.cpp)
  target_link_libraries(${name} PRIVATE ${OpenCV_LIBS})
endforeach()
lecture_example(gauss_newton LeastSquares gauss_newton.cpp)
target_link_libraries(gauss_newton PRIVATE Eigen3::Eigen ${OpenCV_LIBS})
lecture_example(ceres_curve_fitting LeastSquares ceres_curve_fitting.cpp)
target_link_libraries(ceres_curve_fitting PRIVATE Ceres::ceres ${OpenCV_LIBS})
lecture_example(g2o_curve_fitting LeastSquares g2o_curve_fitting.cpp)
target_link_libraries(g2o_curve_fitting PRIVATE core ${OpenCV_LIBS})

lecture_example(feature_training LoopClosureDetection feature_training.cpp)
lecture_example(loop_closure LoopClosureDetection loop_closure.cpp)
lecture_example(gen_vocab LoopClosureDetection gen_vocab_large.cpp)
foreach(name IN ITEMS feature_training loop_closure gen_vocab)
  target_link_libraries(${name} PRIVATE lecture_dbow3)
endforeach()
lecture_example(pose_estimation_3d2d Perspective3Points pose_estimation_3d2d.cpp)
target_include_directories(pose_estimation_3d2d PRIVATE 3rdparty/Sophus)
target_link_libraries(pose_estimation_3d2d PRIVATE core ${OpenCV_LIBS})
lecture_example(triangulation Triangulation triangulation.cpp)
target_link_libraries(triangulation PRIVATE ${OpenCV_LIBS})
lecture_example(test_triangulation Triangulation_opengv test_triangulation.cpp
                random_generators.cpp experiment_helpers.cpp time_measurement.cpp)
target_link_libraries(test_triangulation PRIVATE lecture_opengv)

enable_testing()
include(cmake/LectureTests.cmake)
