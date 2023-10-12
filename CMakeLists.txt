cmake_minimum_required(VERSION 3.22.1)

project(vslam)
set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET)

# TODO: remove this - add includes to individual targets. 
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(vo_motion_estimator src/vo_motion_estimator.cpp)
target_link_libraries(vo_motion_estimator gtsam ${CERES_LIBRARIES})

add_subdirectory(test)