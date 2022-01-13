#include "apriltag_ekf_slam/utilities.h"

namespace utilities{

  tf2::Transform GetTfForced(const std::string& parent, const std::string& child, const tf2_ros::Buffer& tf_buffer, const ros::Time& time, double sleep_time_sec){
    bool success = false;
    tf2::Transform tf;
    geometry_msgs::TransformStamped tf_msg;

    while(!success){
      try{
        tf_msg = tf_buffer.lookupTransform(parent, child, time);
        success = true;
      }catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(sleep_time_sec).sleep();
        success = false;
      } // catch
    } // while(!success)
    tf2::fromMsg(tf_msg.transform, tf);
    return tf;
  } // tf2::Transform GetTfForced
} // namespace utilities
