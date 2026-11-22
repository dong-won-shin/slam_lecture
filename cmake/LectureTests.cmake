function(example_test name)
  set(output "${CMAKE_BINARY_DIR}/results/${name}")
  file(MAKE_DIRECTORY "${output}")
  add_test(NAME ${name} COMMAND ${name} ${ARGN})
  set_tests_properties(${name} PROPERTIES WORKING_DIRECTORY "${output}"
    ENVIRONMENT "SLAM_HEADLESS=1" TIMEOUT 300)
endfunction()

set(projection "${PROJECT_SOURCE_DIR}/3DPointCameraProjection/data")
example_test(3d_point_camera_projection "${projection}/image/0000000000.png"
  "${projection}/dat/C51_LidarPts_0000.dat")
set(cloud "${PROJECT_SOURCE_DIR}/3DPointCloudProcessing/data")
example_test(pass_through_filter "${cloud}/ptf_data.pcd")
example_test(voxel_grid_filter "${cloud}/vgf_data.pcd")
example_test(statistical_outlier_filter "${cloud}/sof_data.pcd")
example_test(normal_estimation "${cloud}/ne_data.pcd")
example_test(clustering "${cloud}/clustering_data.pcd")

set(reg "${PROJECT_SOURCE_DIR}/3DPointCloudRegistration/data")
example_test(iterative_form_registration "${reg}/room_scan1.pcd" "${reg}/room_scan2.pcd")
example_test(closed_form_registration "${reg}/1.png" "${reg}/2.png" "${reg}/1_depth.png" "${reg}/2_depth.png")
example_test(bundle_adjustment_ceres "${PROJECT_SOURCE_DIR}/BundleAdjustment/data/problem-16-22106-pre.txt")
example_test(pose_graph_g2o_SE3 "${PROJECT_SOURCE_DIR}/GraphOptimization/data/sphere.g2o")

set(two_view "${PROJECT_SOURCE_DIR}/Fundamental&EssentialMatrix/data")
example_test(pose_estimation_2d2d "${two_view}/1.png" "${two_view}/2.png")
set(keypoints "${PROJECT_SOURCE_DIR}/Keypoint&Descriptor/data")
example_test(orb_cv "${keypoints}/1.png" "${keypoints}/2.png")
example_test(orb_self "${keypoints}/1.png" "${keypoints}/2.png")
foreach(name IN ITEMS kalman-test gauss_newton ceres_curve_fitting g2o_curve_fitting test_triangulation)
  example_test(${name})
endforeach()

set(loop "${PROJECT_SOURCE_DIR}/LoopClosureDetection/data")
example_test(feature_training "${loop}/")
example_test(loop_closure "${loop}/vocab_larger.yml.gz" "${loop}/")
example_test(gen_vocab "${reg}")
set(pnp "${PROJECT_SOURCE_DIR}/Perspective3Points/data")
example_test(pose_estimation_3d2d "${pnp}/1.png" "${pnp}/2.png" "${pnp}/1_depth.png" "${pnp}/2_depth.png")
set(tri "${PROJECT_SOURCE_DIR}/Triangulation/data")
example_test(triangulation "${tri}/1.png" "${tri}/2.png")

# Some libraries report an unusable result while returning a normal process exit.
set_tests_properties(iterative_form_registration PROPERTIES FAIL_REGULAR_EXPRESSION "has converged:0")
set_tests_properties(bundle_adjustment_ceres ceres_curve_fitting PROPERTIES
  FAIL_REGULAR_EXPRESSION "Termination:[ ]+(FAILURE|USER_FAILURE|NO_CONVERGENCE)")
set_tests_properties(gen_vocab feature_training PROPERTIES FAIL_REGULAR_EXPRESSION "Number of words = 0")
