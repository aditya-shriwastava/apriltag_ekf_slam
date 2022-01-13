#include <ros/ros.h>

#include "apriltag_ekf_slam/apriltag_ekf_slam.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "apriltag_ekf_slam");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  state_estimation::ApriltagEkfSlam apriltag_ekf_slam
  (nh, nh_private);
  ros::spin();
  return 0;
}
