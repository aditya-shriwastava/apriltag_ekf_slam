#pragma once

/////// C++ Standard Template Library ///////
/////////////////////////////////////////////
#include <vector>
#include <string>
#include <cmath>
#include <cstdlib>
/////////////////////////////////////////////

/////// Eigen (Linear Algebra Library) ///////
/////////////////////////////////////////////
#include <Eigen/Dense>
#include <Eigen/LU>
/////////////////////////////////////////////

/////////// ROS Standard Libraries ///////////
/////////////////////////////////////////////
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
/////////////////////////////////////////////

///////// For ApriltagDetection.msg /////////
/////////////////////////////////////////////
#include "apriltag_ekf_slam_msgs/ApriltagDetection.h"
/////////////////////////////////////////////

// Files from apriltag_ekf_slam ros package
/////////////////////////////////////////////
#include "apriltag_ekf_slam/state_belief.h"
#include "apriltag_ekf_slam/utilities.h"
/////////////////////////////////////////////

namespace state_estimation{
  class ApriltagEkfSlam{
  public:
    ApriltagEkfSlam(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  protected:
    struct OdomInput{
      double trans_x = 0;
      double trans_y = 0;
      double rot = 0;
    };

    struct SensorMeasurement{
      int tag_id;
      Eigen::Vector3d mu;
      Eigen::Matrix3d sigma;
    };
    typedef std::vector<ApriltagEkfSlam::SensorMeasurement> SensorMeasurements;

    ros::NodeHandle& _nh;
    ros::NodeHandle& _nh_private;
    bool _debug;

    ros::NodeHandle _nh_apriltag;
    ros::CallbackQueue _apriltag_callback_queue;

    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    tf2_ros::TransformBroadcaster _tf_bdcstr;

    ros::Timer _state_update_timer;
    double _state_update_frequency;

    ros::Subscriber _apriltag_sub;
    ros::Publisher _robot_pose_pub;

    std::string _odom_frame;
    std::string _base_footprint_frame;
    std::string _map_frame;

    ////// Motion Model Parameters //////
    ////////////////////////////////////
    double _sigma_xy_min;
    double _sigma_th_min;
    double _alpha1;
    double _alpha2;
    double _alpha3;
    double _alpha4;
    ////////////////////////////////////

    bool LoadParams();

    void StateUpdateCb(const ros::TimerEvent& e);

    void ApriltagDetectedCb(const apriltag_ekf_slam_msgs::ApriltagDetection::ConstPtr& apriltag_detected);

    ////// Ekf Update Preparation //////
    ////////////////////////////////////

    //! @brief It gives change in robot pose from the previous call.
    //!
    //! It assumes that robot starts at x: 0, y: 0 and yaw = 0.
    //! But if you want it to use the true starting pose of robot as in tf_buffer,
    //! then in the constructor call it once and just through the result.
    //! By doing so, first OdomInput will just contain difference of start robot pose and zero pose
    //! and from then on OdomInput will be as expected.
    ApriltagEkfSlam::OdomInput GetOdomInput(const ros::Time& time);
    ApriltagEkfSlam::SensorMeasurements GetSensorMeasurements(const apriltag_ekf_slam_msgs::ApriltagDetection::ConstPtr& apriltag_detected) const;
    ////////////////////////////////////


    //////////// Ekf Update ////////////
    ////////////////////////////////////
    state_estimation::StateBelief _state_belief;
    void EkfUpdate(const ApriltagEkfSlam::OdomInput& u, const ApriltagEkfSlam::SensorMeasurements& z);
    ////////////////////////////////////


    ///////// Prediction Update /////////
    ////////////////////////////////////
    void PredictionUpdate(const ApriltagEkfSlam::OdomInput& u);
    inline void GetMotionCov(const ApriltagEkfSlam::OdomInput& u, Eigen::Matrix3d& R);
    inline void GetMotionModelJacobian(const ApriltagEkfSlam::OdomInput& u, Eigen::Matrix3d& G);
    inline void UpdatePredictedMu(const ApriltagEkfSlam::OdomInput& u);
    inline void UpdatePredictedCovariance(const Eigen::Matrix3d& R, const Eigen::Matrix3d& G);
    ////////////////////////////////////

    ///////// Correction Update /////////
    ////////////////////////////////////
    void CorrectionUpdate(const ApriltagEkfSlam::SensorMeasurement& z);
    inline Eigen::Vector3d GetTagMu(const ApriltagEkfSlam::SensorMeasurement& z);
    inline void GetMeasurementModelJacobian(Eigen::MatrixXd& H, const ApriltagEkfSlam::SensorMeasurement& z);
    inline void GetKalmanGain(const Eigen::MatrixXd& Q, Eigen::MatrixXd& H, Eigen::MatrixXd& K, const SensorMeasurement& z);
    inline void UpdateCorrectedMu(const Eigen::MatrixXd& K, const ApriltagEkfSlam::SensorMeasurement& z);
    inline Eigen::Vector3d PredictMeasurement(int tag_id);
    inline void UpdateCorrectedCovariance(const Eigen::MatrixXd& K, Eigen::MatrixXd& H, const ApriltagEkfSlam::SensorMeasurement& z);
    ////////////////////////////////////

    /////// Publish State Belief ///////
    ////////////////////////////////////
    void PublishStateBelief();
    void PublishMapTfOdom();
    void PublishRobotPose();
    void PublishTagTf();
    ////////////////////////////////////
  };
}
