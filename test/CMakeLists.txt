find_package(GTest REQUIRED)

add_executable(test_vslam_utils test_vslam_utils.cpp)
target_link_libraries(test_vslam_utils GTest::GTest gtsam)

add_executable(test_vo_motion_estimator test_vo_motion_estimator.cpp)
target_link_libraries(test_vo_motion_estimator GTest::GTest vo_motion_estimator gtsam ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_stereo_traj_generator test_stereo_traj_generator.cpp)
target_link_libraries(test_stereo_traj_generator GTest::GTest stereo_traj_generator gtsam)