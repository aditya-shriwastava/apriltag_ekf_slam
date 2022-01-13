#include "apriltag_ekf_slam/apriltag_detector.h"

namespace apriltag{

  ApriltagDetector::ApriltagDetector(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
  : _nh{nh},
    _nh_private{nh_private},
    _intrinsics_set{false},
    _camera_optical_frame_id_provided{true}{

    if(!this->LoadParams()){
      std::exit(EXIT_FAILURE);
    }

    this->_image_raw_sub = _nh.subscribe("camera/image_raw",
      10,
      &ApriltagDetector::ImageRawCb,
      this);

    this->_camera_info_sub = _nh.subscribe("camera/camera_info",
      10,
      &ApriltagDetector ::CameraInfoCb,
      this);

    if(this->_debug){
      this->_debug_img_pub =
      this->_nh.advertise<sensor_msgs::Image>
      ("apriltag_detected/debug_image",10);
    }

    this->_apriltag_pub =
    this->_nh.advertise<apriltag_ekf_slam_msgs::ApriltagDetection>
    ("apriltag_detected",10);

    if(this->_tag_family != "36h11"){
      ROS_FATAL("Only tag_family 36h11 is supported!");
      std::exit(EXIT_FAILURE);
    }

    this->_tag_detector = apriltag_detector_create();
    this->_apriltag_family = tag36h11_create();
    apriltag_detector_add_family(this->_tag_detector,
    this->_apriltag_family);

  } // ApriltagDetector::ApriltagDetector

  bool ApriltagDetector::LoadParams(){
    if(!this->_nh_private.getParam("tag_family",
    this->_tag_family)){
      ROS_WARN("~/tag_family does not exist");
      this->_tag_family = "36h11";
    }

    if(!this->_nh_private.getParam("tag_size",
    this->_tag_size)){
      ROS_FATAL("~/tag_size does not exist");
      return false;
    }

    if(!this->_nh_private.getParam("pose_covariance_diagonal",
    this->_pose_covariance_diagonal)){
      ROS_FATAL("~/pose_covariance_diagonal does not exist");
      return false;
    }

    if(!this->_nh_private.getParam("debug",
    this->_debug)){
      ROS_WARN("~/debug does not exist");
      this->_debug = false;
    }

    if(!this->_nh_private.getParam("camera_optical_frame_id",
    this->_camera_optical_frame_id)){
      ROS_WARN("~/camera_optical_frame_id does not exist");
      this->_camera_optical_frame_id_provided = false;
    }
    return true;
  } // ApriltagDetector::LoadParams

  void ApriltagDetector::ImageRawCb(
  const sensor_msgs::Image::ConstPtr& image_raw){
    if(!this->_intrinsics_set){
      ROS_WARN("Intrinsic parameters not set");
      return;
    }

    cv_bridge::CvImageConstPtr image_raw_cvptr;
    image_raw_cvptr = cv_bridge::toCvShare(image_raw, image_raw->encoding);

    cv_bridge::CvImagePtr image_raw_grayscale_cvptr =
    cv_bridge::cvtColor(image_raw_cvptr, "mono8");

    image_u8_t image_raw_u8 =
    {   .width = image_raw_grayscale_cvptr->image.cols,
        .height = image_raw_grayscale_cvptr->image.rows,
        .stride = image_raw_grayscale_cvptr->image.cols,
        .buf = image_raw_grayscale_cvptr->image.data
    };

    zarray_t *detections = apriltag_detector_detect
    (this->_tag_detector, &image_raw_u8);

    static int sequence_id = 0;
    if( zarray_size(detections) > 0){
      apriltag_ekf_slam_msgs::ApriltagDetection tag_detection_msg;
      tag_detection_msg.header.seq = sequence_id++;
      tag_detection_msg.header.stamp = ros::Time::now();
      tag_detection_msg.header.frame_id =
      this->_camera_optical_frame_id;

      for(int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = this->_tag_size;
        info.fx = this->_fx;
        info.fy = this->_fy;
        info.cx = this->_cx;
        info.cy = this->_cy;

        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info, &pose);

        if( (det->hamming > 0) || (err > 0.001) ){ //0.00001
          ROS_WARN("Tag%d:: corrected_hamming: %d, tf_error: %f rejeced!!",
          det->id, det->hamming, err);
        }else{
          this->AppendTagDetected(&tag_detection_msg, det, &pose);
          if(this->_debug){
            this->AnnotateDetection
            (image_raw_cvptr, det);
          } // if(this->_debug){
        } //}else{

        matd_destroy(pose.R);
        matd_destroy(pose.t);
      } // for
      this->_apriltag_pub.publish(tag_detection_msg);

      if(this->_debug){
        this->_debug_img_pub.publish
        (image_raw_cvptr->toImageMsg());

        this->PublishTagTf(tag_detection_msg);
      }
    } // if( zarray_size(detections) > 0)
  } // ApriltagDetector::ImageRawCb

  void ApriltagDetector::CameraInfoCb(
  const sensor_msgs::CameraInfo::ConstPtr& camera_info){
    this->_cx = camera_info->K[2];
    this->_cy = camera_info->K[5];
    this->_fx = camera_info->K[0];
    this->_fy = camera_info->K[4];

    if(!this->_camera_optical_frame_id_provided){
      this->_camera_optical_frame_id =
      camera_info->header.frame_id + "_optical";
    }

    if(!this->_intrinsics_set){
      this->_intrinsics_set = true;
      ROS_DEBUG("Intrinsic parameters set");
    }
  } // ApriltagDetector::CameraInfoCb

  void ApriltagDetector::AppendTagDetected(
  apriltag_ekf_slam_msgs::ApriltagDetection* tag_detection_msg,
  apriltag_detection_t* detection,
  apriltag_pose_t* detection_pose){
    tf2::Matrix3x3 tag_rmat(
      detection_pose->R->data[0],
      detection_pose->R->data[1],
      detection_pose->R->data[2],
      detection_pose->R->data[3],
      detection_pose->R->data[4],
      detection_pose->R->data[5],
      detection_pose->R->data[6],
      detection_pose->R->data[7],
      detection_pose->R->data[8]
    );

    tf2::Vector3 tag_origin(
      detection_pose->t->data[0],
      detection_pose->t->data[1],
      detection_pose->t->data[2]
    );

    tf2::Transform tag_tf;
    tag_tf.setBasis(tag_rmat);
    tag_tf.setOrigin(tag_origin);

    std_msgs::UInt32 tag_id;
    tag_id.data = detection->id;
    tag_detection_msg->tag_id.emplace_back(tag_id);

    geometry_msgs::PoseWithCovariance tag_pose;
    tf2::toMsg(tag_tf, tag_pose.pose);
    for(int i=0; i<6; i++){
      tag_pose.covariance.at(7 * i) =
      this->_pose_covariance_diagonal.at(i);
    }
    tag_detection_msg->pose.emplace_back(tag_pose);
  } // ApriltagDetector::AppendTagDetected

  void ApriltagDetector::PublishTagTf(
  const apriltag_ekf_slam_msgs::ApriltagDetection& tag_detection_msg){
    geometry_msgs::TransformStamped tf;

    static int seq_id = 0;
    tf.header.seq = seq_id++;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = tag_detection_msg.header.frame_id;

    for(int i=0; i<tag_detection_msg.tag_id.size(); i++){
      tf.child_frame_id = "tag_" +
      std::to_string(tag_detection_msg.tag_id.at(i).data);

      tf.transform.rotation =
      tag_detection_msg.pose.at(i).pose.orientation;
      tf.transform.translation.x =
      tag_detection_msg.pose.at(i).pose.position.x;
      tf.transform.translation.y =
      tag_detection_msg.pose.at(i).pose.position.y;
      tf.transform.translation.z =
      tag_detection_msg.pose.at(i).pose.position.z;

      this->_tf_bdcstr.sendTransform(tf);
    } // for(int i=0; i<tag_detection_msg.tag_id.size(); i++)
  } // ApriltagDetector::PublishTagTf

  void ApriltagDetector::AnnotateDetection(
  cv_bridge::CvImageConstPtr image_raw_cvptr,
  apriltag_detection_t* detection){
    cv::line(image_raw_cvptr->image,
      cv::Point((int)detection->p[0][0], (int)detection->p[0][1]),
      cv::Point((int)detection->p[1][0], (int)detection->p[1][1]),
      cv::Scalar(0, 0xff, 0), 4
    ); // green

    cv::line(image_raw_cvptr->image,
      cv::Point((int)detection->p[0][0], (int)detection->p[0][1]),
      cv::Point((int)detection->p[3][0], (int)detection->p[3][1]),
      cv::Scalar(0, 0, 0xff), 4
    ); // blue

    cv::line(image_raw_cvptr->image,
      cv::Point((int)detection->p[1][0], (int)detection->p[1][1]),
      cv::Point((int)detection->p[2][0], (int)detection->p[2][1]),
      cv::Scalar(0xff, 0, 0), 4
    ); // red

    cv::line(image_raw_cvptr->image,
      cv::Point((int)detection->p[2][0], (int)detection->p[2][1]),
      cv::Point((int)detection->p[3][0], (int)detection->p[3][1]),
      cv::Scalar(0xff, 0, 0), 4
    ); // red
  } // ApriltagDetector::AnnotateDetection

} // namespace apriltag
