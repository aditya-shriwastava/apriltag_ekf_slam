cmake_minimum_required(VERSION 2.8.3)
project(apriltag_ekf_slam_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
 FILES
 ApriltagDetection.msg
)

generate_messages(
 DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
)
