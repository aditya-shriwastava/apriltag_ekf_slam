#include "apriltag_ekf_slam/apriltag_ekf_slam.h"

namespace state_estimation{
  ApriltagEkfSlam::ApriltagEkfSlam
  (ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : _nh{nh},
    _nh_private{nh_private},
    _tf_listener(_tf_buffer){

    if(!this->LoadParams()){
      std::exit(EXIT_FAILURE);
    }

    this->GetOdomInput(ros::Time::now());
    
    this->_state_update_timer = this->_nh.createTimer(ros::Rate(this->_state_update_frequency), &ApriltagEkfSlam::StateUpdateCb, this);

    this->_nh_apriltag.setCallbackQueue(&this->_apriltag_callback_queue);
    this->_apriltag_sub = this->_nh_apriltag.subscribe("apriltag_detected", 10, &ApriltagEkfSlam::ApriltagDetectedCb, this);
    this->_robot_pose_pub =this-> _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot_pose", 1);
  }

  bool ApriltagEkfSlam::LoadParams(){
    if(!this->_nh_private.getParam("state_update_frequency", this->_state_update_frequency)){
      ROS_WARN("state_update_frequency is not set (Default: 10hz)");
      this->_state_update_frequency = 10;
    }

    if(!this->_nh_private.getParam("debug", this->_debug)){
      ROS_WARN("debug is not set (Default: false)");
      this->_debug = false;
    }

    if(!this->_nh_private.getParam("Frames/odom", this->_odom_frame)){
      ROS_WARN("Frames/odom is not set (Default: odom)");
      this->_odom_frame = "odom";
    }

    if(!this->_nh_private.getParam("Frames/base_footprint", this->_base_footprint_frame)){
      ROS_WARN("Frames/base_footprint is not set (Default: base_footprint)");
      this->_base_footprint_frame = "base_footprint";
    }

    if(!this->_nh_private.getParam("Frames/map", this->_map_frame)){
      ROS_WARN("Frames/map is not set (Default: map)");
      this->_map_frame = "map";
    }

    if(!this->_nh_private.getParam("MotionModelParameters/sigma_xy_min", this->_sigma_xy_min)){
      ROS_WARN("MotionModelParameters/sigma_xy_min is not set (Default: 0.0)");
      this->_sigma_xy_min = 0.0;
    }

    if(!this->_nh_private.getParam("MotionModelParameters/sigma_th_min", this->_sigma_th_min)){
      ROS_WARN("MotionModelParameters/sigma_th_min is not set (Default: 0.0)");
      this->_sigma_th_min = 0.0;
    }

    if(!this->_nh_private.getParam("MotionModelParameters/alpha1", this->_alpha1)){
      ROS_WARN("MotionModelParameters/alpha1 is not set (Default: 0.002)");
      this->_alpha1 = 0.002;
    }

    if(!this->_nh_private.getParam("MotionModelParameters/alpha2", this->_alpha2)){
      ROS_WARN("MotionModelParameters/alpha2 is not set (Default: 0.0005)");
      this->_alpha2 = 0.0005;
    }

    if(!this->_nh_private.getParam("MotionModelParameters/alpha3", this->_alpha3)){
      ROS_WARN("MotionModelParameters/alpha3 is not set (Default: 0.0005)");
      this->_alpha3 = 0.0005;
    }

    if(!this->_nh_private.getParam("MotionModelParameters/alpha4", this->_alpha4)){
      ROS_WARN("MotionModelParameters/alpha4 is not set (Default: 0.002)");
      this->_alpha4 = 0.002;
    }
    return true;
  } // ApriltagEkfSlam::LoadParams

  void ApriltagEkfSlam::StateUpdateCb(const ros::TimerEvent& e){
    this->_apriltag_callback_queue.callAvailable();

    ApriltagEkfSlam::OdomInput u = this->GetOdomInput(ros::Time::now());
    ApriltagEkfSlam::SensorMeasurements z;
    this->EkfUpdate(u, z);

    this->PublishStateBelief();
  } // ApriltagEkfSlam::StateUpdateCb

  void ApriltagEkfSlam::ApriltagDetectedCb(const apriltag_ekf_slam_msgs::ApriltagDetection::ConstPtr& apriltag_detected){
    ApriltagEkfSlam::OdomInput u = this->GetOdomInput(apriltag_detected->header.stamp);
    ApriltagEkfSlam::SensorMeasurements z = this->GetSensorMeasurements(apriltag_detected);
    this->EkfUpdate(u, z);
  } // ApriltagEkfSlam::ApriltagDetectedCb

  ApriltagEkfSlam::OdomInput ApriltagEkfSlam::GetOdomInput(const ros::Time& time){
    static tf2::Transform _srobot_tf(tf2::Quaternion(0,0,0,1), tf2::Vector3(0,0,0));

    tf2::Transform robot_tf = utilities::GetTfForced(this->_odom_frame, this->_base_footprint_frame, this->_tf_buffer, time, 0.05);

    tf2::Transform odom_tf;
    odom_tf = _srobot_tf.inverse() * robot_tf;

    ApriltagEkfSlam::OdomInput u;
    u.trans_x = odom_tf.getOrigin().x();
    u.trans_y = odom_tf.getOrigin().y();
    double roll, pitch, yaw;
    odom_tf.getBasis().getRPY(roll, pitch, yaw);
    u.rot = yaw;

    _srobot_tf = robot_tf;
    return u;
  } // ApriltagEkfSlam::OdomInput ApriltagEkfSlam::GetOdomInput

  ApriltagEkfSlam::SensorMeasurements ApriltagEkfSlam::GetSensorMeasurements(const apriltag_ekf_slam_msgs::ApriltagDetection::ConstPtr& apriltag_detected) const{
    tf2::Transform camera_optical_frame_tf = utilities::GetTfForced(this->_base_footprint_frame, apriltag_detected->header.frame_id, this->_tf_buffer, apriltag_detected->header.stamp, 0.05);
    ApriltagEkfSlam::SensorMeasurements z;

    for(int i=0; i<apriltag_detected->tag_id.size(); i++){
      ApriltagEkfSlam::SensorMeasurement zi;
      zi.tag_id = apriltag_detected->tag_id.at(i).data;

      tf2::Transform tag_tf;
      tf2::fromMsg(apriltag_detected->pose.at(i).pose, tag_tf); // tag_tf in camera_optical_frame
      tag_tf = camera_optical_frame_tf * tag_tf; // tag_tf in _base_footprint_frame

      zi.mu.x() = tag_tf.getOrigin().x();
      zi.mu.y() = tag_tf.getOrigin().y();
      zi.mu.z() = tag_tf.getOrigin().z();

      for(int row=0; row<3; row++){
        for(int col=0; col<3; col++){
          zi.sigma(row,col) = apriltag_detected->pose.at(i).covariance.at(col + (row * 6));
        } // for(int y=0; y<3; y++)
      } // for(int x=0; x<3; x++)

      z.emplace_back(zi);
    } // for(int i=0; i<apriltag_detected->tag_id.size(); i++)
    return z;
  } // ApriltagEkfSlam::SensorMeasurements ApriltagEkfSlam::GetSensorMeasurements

  void ApriltagEkfSlam::EkfUpdate(const ApriltagEkfSlam::OdomInput& u, const ApriltagEkfSlam::SensorMeasurements& z){
    this->PredictionUpdate(u);
    for(auto& zi: z){
      this->CorrectionUpdate(zi);
    } // for(auto& zi: z)
  } // ApriltagEkfSlam::EkfUpdate

  void ApriltagEkfSlam::PredictionUpdate(const ApriltagEkfSlam::OdomInput& u){
    Eigen::Matrix3d R, G;
    R = Eigen::Matrix3d::Zero(3, 3);
    G = Eigen::Matrix3d::Zero(3, 3);
    this->GetMotionCov(u, R);
    this->GetMotionModelJacobian(u, G);

    this->UpdatePredictedMu(u);
    this->UpdatePredictedCovariance(R, G);
  } // ApriltagEkfSlam::PredictionUpdate

  inline void ApriltagEkfSlam::GetMotionCov(const ApriltagEkfSlam::OdomInput& u, Eigen::Matrix3d& R){
     R(0,0) = this->_sigma_xy_min
              + (this->_alpha1 * std::sqrt(std::pow(u.trans_x,2)+
                                           std::pow(u.trans_y,2)))
              + (this->_alpha2 * std::abs(u.rot));
     R(1,1) = R(0,0);
     R(2,2) = this->_sigma_th_min
              + (this->_alpha3 * std::sqrt(std::pow(u.trans_x,2)+
                                           std::pow(u.trans_y,2)))
              + (this->_alpha4 * std::abs(u.rot));
   } // ApriltagEkfSlam::GetMotionCov

   inline void ApriltagEkfSlam::GetMotionModelJacobian(const ApriltagEkfSlam::OdomInput& u, Eigen::Matrix3d& G){
     Eigen::VectorBlock<Eigen::VectorXd> mu_x = this->_state_belief.mu_x();

     G(0,0) = 1;
     G(0,2) = -( (u.trans_x * std::sin(mu_x(2))) +
                 (u.trans_y * std::cos(mu_x(2))) );
     G(1,1) = 1;
     G(1,2) = ( (u.trans_x * std::cos(mu_x(2))) -
                (u.trans_y * std::sin(mu_x(2))) );
     G(2,2) = 1;
   } // ApriltagEkfSlam::GetMotionModelJacobian

  inline void ApriltagEkfSlam::UpdatePredictedMu(const ApriltagEkfSlam::OdomInput& u){
    Eigen::VectorBlock<Eigen::VectorXd> mu_x = this->_state_belief.mu_x();

    mu_x(0) = mu_x(0) +
              ( u.trans_x * std::cos(mu_x(2)) ) -
              ( u.trans_y * std::sin(mu_x(2)) );
    mu_x(1) = mu_x(1) +
              ( u.trans_x * std::sin(mu_x(2)) ) +
              ( u.trans_y * std::cos(mu_x(2)) );
    mu_x(2) = mu_x(2) + u.rot;
  } // ApriltagEkfSlam::UpdatePredictedMu

  inline void ApriltagEkfSlam::UpdatePredictedCovariance(const Eigen::Matrix3d& R, const Eigen::Matrix3d& G){
    Eigen::Block<Eigen::MatrixXd> sigma_xx = this->_state_belief.sigma_xx();
    sigma_xx = ( G * sigma_xx * G.transpose() ) + R;

    if(this->_state_belief.GetSize() > 3){
      Eigen::Block<Eigen::MatrixXd> sigma_xm = this->_state_belief.sigma_xm();
      Eigen::Block<Eigen::MatrixXd> sigma_mx = this->_state_belief.sigma_mx();

      sigma_xm = G * sigma_xm;
      sigma_mx = sigma_mx * G.transpose();
    } // if(this->_state_belief.GetSize() > 3)
  } // ApriltagEkfSlam::UpdatePredictedCovariance

   void ApriltagEkfSlam::CorrectionUpdate(const ApriltagEkfSlam::SensorMeasurement& z){

     if( !this->_state_belief.IsTracked(z.tag_id) ){
       this->_state_belief.NewTag(z.tag_id, this->GetTagMu(z));
     }

     Eigen::MatrixXd Q = z.sigma; // Measurement Covariance
     Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
     this->GetMeasurementModelJacobian(H, z);

     Eigen::MatrixXd K = Eigen::MatrixXd::Zero(this->_state_belief.GetSize(), 3);
     this->GetKalmanGain(Q, H, K, z);
     this->UpdateCorrectedMu(K, z);
     this->UpdateCorrectedCovariance(K, H, z);
   } // ApriltagEkfSlam::CorrectionUpdate

   inline Eigen::Vector3d ApriltagEkfSlam::GetTagMu(const ApriltagEkfSlam::SensorMeasurement& z){
     Eigen::Vector3d mu_tag;
     double mu_x = this->_state_belief.mu_x()(0);
     double mu_y = this->_state_belief.mu_x()(1);
     double mu_th = this->_state_belief.mu_x()(2);
     mu_tag.x() = mu_x +
                  ( z.mu.x() * std::cos(mu_th) ) -
                  ( z.mu.y() * std::sin(mu_th) );
     mu_tag.y() = mu_y +
                  ( z.mu.x() * std::sin(mu_th) ) +
                  ( z.mu.y() * std::cos(mu_th) );
     mu_tag.z() = z.mu.z();
     return mu_tag;
   } // ApriltagEkfSlam::GetTagMu

   inline void ApriltagEkfSlam::GetMeasurementModelJacobian(Eigen::MatrixXd& H, const ApriltagEkfSlam::SensorMeasurement& z){

     Eigen::VectorBlock<Eigen::VectorXd> mu_x = this->_state_belief.mu_x();
     Eigen::VectorBlock<Eigen::VectorXd> mu_m = this->_state_belief.mu_m(z.tag_id);

     H(0,0) = -std::cos(mu_x(2));
     H(0,1) = -std::sin(mu_x(2));
     H(0,2) = ( (mu_m(1) - mu_x(1)) * std::cos(mu_x(2)) ) -
                 ( (mu_m(0) - mu_x(0)) * std::sin(mu_x(2)) );
     H(0,3) = std::cos(mu_x(2));
     H(0,4) = std::sin(mu_x(2));
     H(0,5) = 0;

     H(1,0) = std::sin(mu_x(2));
     H(1,1) = -std::cos(mu_x(2));
     H(1,2) = -( ( (mu_m(1) - mu_x(1)) * std::sin(mu_x(2)) ) +
                    ( (mu_m(0) - mu_x(0)) * std::cos(mu_x(2)) ) );
     H(1,3) = -std::sin(mu_x(2));
     H(1,4) = std::cos(mu_x(2));
     H(1,5) = 0;

     H(2,0) = 0;
     H(2,1) = 0;
     H(2,2) = 0;
     H(2,3) = 0;
     H(2,4) = 0;
     H(2,5) = 1;
   } // ApriltagEkfSlam::GetMeasurementModelJacobian

   inline void ApriltagEkfSlam::GetKalmanGain(const Eigen::MatrixXd& Q, Eigen::MatrixXd& H, Eigen::MatrixXd& K, const ApriltagEkfSlam::SensorMeasurement& z){
     Eigen::Block<Eigen::MatrixXd> Hx = H.block(0, 0, 3, 3);
     Eigen::Block<Eigen::MatrixXd> Hm = H.block(0, 3, 3, 3);

     Eigen::Block<Eigen::MatrixXd> sigma_xx = this->_state_belief.sigma_xx();
     Eigen::Block<Eigen::MatrixXd> sigma_mm = this->_state_belief.sigma_mm(z.tag_id, z.tag_id);
     Eigen::Block<Eigen::MatrixXd> sigma_xm = this->_state_belief.sigma_xm(z.tag_id);
     Eigen::Block<Eigen::MatrixXd> sigma_mx = this->_state_belief.sigma_mx(z.tag_id);

     Eigen::Block<Eigen::MatrixXd> sigma_star_x = this->_state_belief.sigma_star_x();
     Eigen::Block<Eigen::MatrixXd> sigma_star_m = this->_state_belief.sigma_star_m(z.tag_id);

     Eigen::MatrixXd H_sigma_Ht, sigma_Ht;

     H_sigma_Ht = ( Hx * ( (sigma_xx * (Hx.transpose())) + (sigma_xm * (Hm.transpose())) ) ) +
                  ( Hm * ( (sigma_mx * (Hx.transpose())) + (sigma_mm * (Hm.transpose())) ) );

     sigma_Ht = ( sigma_star_x * (Hx.transpose()) ) +
                ( sigma_star_m * (Hm.transpose()) );

     K = sigma_Ht * ( H_sigma_Ht + Q ).inverse();
   } // ApriltagEkfSlam::GetKalmanGain

   inline void ApriltagEkfSlam::UpdateCorrectedMu(const Eigen::MatrixXd& K, const ApriltagEkfSlam::SensorMeasurement& z){
     Eigen::VectorBlock<Eigen::VectorXd> mu = this->_state_belief.mu();
     Eigen::Vector3d z_predicted = this->PredictMeasurement(z.tag_id);
     mu = mu + ( K * (z.mu - z_predicted) );
   }

   inline Eigen::Vector3d ApriltagEkfSlam::PredictMeasurement(int tag_id){

     Eigen::VectorBlock<Eigen::VectorXd> mu_x = this->_state_belief.mu_x();
     Eigen::VectorBlock<Eigen::VectorXd> mu_m = this->_state_belief.mu_m(tag_id);

     Eigen::Vector3d z_predicted;
     z_predicted.x() = ( (mu_m(1) - mu_x(1)) * std::sin(mu_x(2)) ) +
                       ( (mu_m(0) - mu_x(0)) * std::cos(mu_x(2)) );
     z_predicted.y() = ( (mu_m(1) - mu_x(1)) * std::cos(mu_x(2)) ) -
                       ( (mu_m(0) - mu_x(0)) * std::sin(mu_x(2)) );
     z_predicted.z() = mu_m(2);

     return z_predicted;
   } // ApriltagEkfSlam::UpdateCorrectedMu

   inline void ApriltagEkfSlam::UpdateCorrectedCovariance(const Eigen::MatrixXd& K, Eigen::MatrixXd& H, const ApriltagEkfSlam::SensorMeasurement& z){

     Eigen::Block<Eigen::MatrixXd> Hx = H.block(0, 0, 3, 3);
     Eigen::Block<Eigen::MatrixXd> Hm = H.block(0, 3, 3, 3);

     Eigen::Block<Eigen::MatrixXd> sigma = this->_state_belief.sigma();

     Eigen::Block<Eigen::MatrixXd> sigma_x_star = this->_state_belief.sigma_x_star();
     Eigen::Block<Eigen::MatrixXd> sigma_m_star = this->_state_belief.sigma_m_star(z.tag_id);

     sigma = sigma - ( K * ( (Hx * sigma_x_star)+
                             (Hm * sigma_m_star) )
                     );
   } // ApriltagEkfSlam::UpdateCorrectedCovariance

   void ApriltagEkfSlam::PublishStateBelief(){
     this->PublishMapTfOdom();
     this->PublishRobotPose();
     if(this->_debug){
       this->PublishTagTf();
     }
   }

   void ApriltagEkfSlam::PublishMapTfOdom(){
     tf2::Transform map_tf_footprint,
                    odom_tf_footprint,
                    map_tf_odom;

      odom_tf_footprint = utilities::GetTfForced(this->_odom_frame, this->_base_footprint_frame, this->_tf_buffer, ros::Time::now(), 0.05);

      Eigen::VectorBlock<Eigen::VectorXd> mu_x = this->_state_belief.mu_x();
      tf2::Vector3 origin(mu_x(0), mu_x(1), 0);
      tf2::Quaternion rotation;
      rotation.setRPY(0,0,mu_x(2));
      map_tf_footprint.setOrigin( origin );
      map_tf_footprint.setRotation( rotation );

      map_tf_odom =  map_tf_footprint * odom_tf_footprint.inverse();

      geometry_msgs::TransformStamped map_tf_odom_msg;
      map_tf_odom_msg.header.stamp = ros::Time::now();
      map_tf_odom_msg.header.frame_id = this->_map_frame;
      map_tf_odom_msg.child_frame_id = this->_odom_frame;
      map_tf_odom_msg.transform = tf2::toMsg(map_tf_odom);
      this->_tf_bdcstr.sendTransform(map_tf_odom_msg);

   } // ApriltagEkfSlam::PublishMapTfOdom

   void ApriltagEkfSlam::PublishRobotPose(){
     geometry_msgs::PoseWithCovarianceStamped pose;

     Eigen::VectorBlock<Eigen::VectorXd> mu_x = this->_state_belief.mu_x();

     pose.header.stamp = ros::Time::now();
     pose.header.frame_id = this->_map_frame;

     pose.pose.pose.position.x = mu_x(0);
     pose.pose.pose.position.y = mu_x(1);
     pose.pose.pose.position.z = 0;

     tf2::Quaternion orientation;
     orientation.setRPY( 0, 0, mu_x(2) );
     pose.pose.pose.orientation.x = orientation.x();
     pose.pose.pose.orientation.y = orientation.y();
     pose.pose.pose.orientation.z = orientation.z();
     pose.pose.pose.orientation.w = orientation.w();

     Eigen::Block<Eigen::MatrixXd> sigma_xx = this->_state_belief.sigma_xx();
     //       x     y    yaw      x     y    yaw
     //  x  (0,0) (0,1) (0,5) = (0,0) (0,1) (0,2)
     //  y  (1,0) (1,1) (1,5) = (1,0) (1,1) (1,2)
     // yaw (5,0) (5,1) (5,5) = (2,0) (2,1) (2,2)
     pose.pose.covariance.at( (0*6) + 0 ) = sigma_xx(0, 0);
     pose.pose.covariance.at( (0*6) + 1 ) = sigma_xx(0, 1);
     pose.pose.covariance.at( (0*6) + 5 ) = sigma_xx(0, 2);
     pose.pose.covariance.at( (1*6) + 0 ) = sigma_xx(1, 0);
     pose.pose.covariance.at( (1*6) + 1 ) = sigma_xx(1, 1);
     pose.pose.covariance.at( (1*6) + 5 ) = sigma_xx(1, 2);
     pose.pose.covariance.at( (5*6) + 0 ) = sigma_xx(2, 0);
     pose.pose.covariance.at( (5*6) + 1 ) = sigma_xx(2, 1);
     pose.pose.covariance.at( (5*6) + 5 ) = sigma_xx(2, 2);

     this->_robot_pose_pub.publish(pose);
   } // ApriltagEkfSlam::PublishRobotPose

   void ApriltagEkfSlam::PublishTagTf(){
     geometry_msgs::TransformStamped tag_tf;

     tag_tf.header.stamp = ros::Time::now();
     tag_tf.header.frame_id = this->_map_frame;

     const std::vector<int>* tags = this->_state_belief.GetTags();
     for(auto& tag_id : *tags){
       Eigen::VectorBlock<Eigen::VectorXd> mu_m  = this->_state_belief.mu_m(tag_id);

       tag_tf.child_frame_id = "tag_" + std::to_string(tag_id);

       tag_tf.transform.translation.x = mu_m(0);
       tag_tf.transform.translation.y = mu_m(1);
       tag_tf.transform.translation.z = mu_m(2);

       tag_tf.transform.rotation.x = 0;
       tag_tf.transform.rotation.y = 0;
       tag_tf.transform.rotation.z = 0;
       tag_tf.transform.rotation.w = 1;

       this->_tf_bdcstr.sendTransform(tag_tf);
     } // for(auto& tag_id : *tags)
   } // ApriltagEkfSlam::PublishTagTf

} // namespace state_estimation
