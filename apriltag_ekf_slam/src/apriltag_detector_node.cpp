#include <ros/ros.h>

#include "apriltag_ekf_slam/apriltag_detector.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "apriltag_detector");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  apriltag::ApriltagDetector apriltag_detector
  (nh, nh_private);
  ros::spin();
  return 0;
}
