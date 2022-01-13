cmake_minimum_required(VERSION 2.8.3)
project(apriltag_ekf_slam)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2
  geometry_msgs
  tf2_geometry_msgs
  apriltag_ekf_slam_msgs
)

find_package( OpenCV REQUIRED )

find_package(Eigen3 REQUIRED)

find_package(PkgConfig)
pkg_search_module(apriltag REQUIRED apriltag)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    cv_bridge
    tf2_ros
    tf2
    geometry_msgs
    tf2_geometry_msgs
    apriltag_ekf_slam_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(state_belief_class
  src/modules/state_belief.cpp
)
target_link_libraries(state_belief_class
  ${catkin_LIBRARIES}
)

add_library(apriltag_ekf_slam_class
  src/modules/apriltag_ekf_slam.cpp
  src/modules/utilities.cpp
)
target_link_libraries(apriltag_ekf_slam_class
  ${catkin_LIBRARIES}
  state_belief_class
)

add_executable(apriltag_ekf_slam
  src/apriltag_ekf_slam_node.cpp
)
target_link_libraries(apriltag_ekf_slam
  ${catkin_LIBRARIES}
  apriltag_ekf_slam_class
)

add_executable(apriltag_detector
  src/apriltag_detector_node.cpp
  src/modules/apriltag_detector.cpp
)
target_link_libraries(apriltag_detector
  ${catkin_LIBRARIES}
  ${apriltag_LIBRARIES}
)

add_executable(apriltag_scale src/apriltag_scale_node.cpp)
target_link_libraries(apriltag_scale
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_state_belief
    test/state_belief/test_state_belief.test
    test/state_belief/test_state_belief.cpp
  )

  target_link_libraries(test_state_belief
    ${catkin_LIBRARIES}
    state_belief_class
  )

  add_rostest_gtest(test_apriltag_ekf_slam
    test/apriltag_ekf_slam/test_apriltag_ekf_slam.test
    test/apriltag_ekf_slam/test_apriltag_ekf_slam.cpp
  )
  target_link_libraries(test_apriltag_ekf_slam
    ${catkin_LIBRARIES}
    apriltag_ekf_slam_class
  )
endif()
