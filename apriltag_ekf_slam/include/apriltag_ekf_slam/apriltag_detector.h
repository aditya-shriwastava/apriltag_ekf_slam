#pragma once

// C++ Standard Template Library
#include <string>
#include <cstdlib>
////////////////////////////////

// ROS Standard Libraries
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
////////////////////////////////

// Apriltag C Libraries
// Github: https://github.com/AprilRobotics/apriltag
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag36h11.h>
////////////////////////////////

// For ApriltagDetection.msg
#include "apriltag_ekf_slam_msgs/ApriltagDetection.h"

namespace apriltag{
  //! @brief It subscriber to a camera, finds apriltags in the
  //! image and publishes information about the tags.
  class ApriltagDetector{
  public:
    ApriltagDetector(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  private:
    ros::NodeHandle _nh, _nh_private;
    ros::Subscriber _image_raw_sub, _camera_info_sub;
    ros::Publisher _apriltag_pub;
    ros::Publisher _debug_img_pub;
    tf2_ros::TransformBroadcaster _tf_bdcstr;

    bool LoadParams();

    //! @brief Image callback
    //!
    //! ## Psudo code
    //! 1. Check if intrinsics are set. If not then return.
    //! 2. Convert image_raw to image_u8_t
    //!   - As this is the image format supported by apriltag
    //!     detector.
    //! 3. Detect all the tags in the image
    //! 4. For every tag in the detection:
    //!   1. Get pose of the tag
    //!   2. Append tag dectected to tag_detection_msg
    //!     - msg format: apriltag_ekf_slam::ApriltagDetection
    //!   3. Put bounding-box around the detected tag (If Debug enabled)
    //! 5. Publish tag_detection_msg
    //! 6. If Debug is enabled:
    //!   1. Publish debug image
    //!   2. Publish all the tf of tags detected.
    void ImageRawCb(const sensor_msgs::Image::ConstPtr& image_raw);

    //! ## Psudo code
    //! 1. Save the intrinsics in _cx, _cy, _fx, _fy from
    //! camera info.
    //! 2. If camera_optical_frame_id is not set in parameter
    //! server then set it from the camera_info msg
    void CameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& camera_info);

    void AppendTagDetected(
      apriltag_ekf_slam_msgs::ApriltagDetection* tag_detection_msg,
      apriltag_detection_t* detection,
      apriltag_pose_t* detection_pose);

    void PublishTagTf(
      const apriltag_ekf_slam_msgs::ApriltagDetection& tag_detection_msg);

    //! @brief Draws bounding box around the detected tags in the
    //! color image.
    //!
    //! @param[out] image_raw_cvptr Pointer to the image we want
    //! to add bounding boxes to.
    //! @param[in] detection Holds information about the tag
    //! detected
    void AnnotateDetection(
      cv_bridge::CvImageConstPtr image_raw_cvptr,
      apriltag_detection_t* detection);

    apriltag_detector_t* _tag_detector;
    apriltag_family_t* _apriltag_family;

    bool _intrinsics_set;
    double _fx, _fy, _cx, _cy;

    std::string _tag_family;

    double _tag_size;

    bool _debug;

    bool _camera_optical_frame_id_provided;
    std::string _camera_optical_frame_id;

    std::vector<double> _pose_covariance_diagonal;
  }; // class ApriltagDetector

} // namespace apriltag
