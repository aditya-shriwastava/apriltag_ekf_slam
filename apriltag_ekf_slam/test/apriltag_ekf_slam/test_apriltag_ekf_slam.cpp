#include <gtest/gtest.h>
#include <ros/ros.h>
#include "apriltag_ekf_slam/apriltag_ekf_slam.h"

TEST(Dummy, Equate){
  EXPECT_EQ(2,2);
}

// class TestApriltagEkfSlam : public state_estimation::ApriltagEkfSlam{
// public:
//   state_estimation::StateBelief* GetStateBelief_(){
//     return &_state_belief;
//   }
//
//   state_estimation::OdomInput GetOdomInput_(tf2::Transform* robot_tf,
//                                tf2::Transform* robot_tf_new){
//     return this->GetOdomInput(robot_tf, robot_tf_new);
//   }
//
//   void GetMotionCov_(state_estimation::OdomInput* u,
//                      Eigen::Matrix3d* R){
//     this->GetMotionCov(u, R);
//   }
//
//   void SetMotonModelParams_(double sigma_xy_min,
//                             double sigma_th_min,
//                             double alpha1,
//                             double alpha2,
//                             double alpha3,
//                             double alpha4){
//      this->_sigma_xy_min = sigma_xy_min;
//      this->_sigma_th_min = sigma_th_min;
//      this->_alpha1 = alpha1;
//      this->_alpha2 = alpha2;
//      this->_alpha3 = alpha3;
//      this->_alpha4 = alpha4;
//    }
//
//    void GetMotionModelJacobian_(state_estimation::OdomInput* u,
//                                 Eigen::Matrix3d* G){
//      this->GetMotionModelJacobian(u, G);
//    }
//
//     void UpdatePredictedMu_(state_estimation::OdomInput* u){
//       this->UpdatePredictedMu(u);
//     }
//
//     Eigen::Vector3d GetTagMu_(state_estimation::SensorMeasurement* z){
//       return this->GetTagMu(z);
//     }
// };
//
// TEST(EkfSetup, GetOdomInput){
//   TestApriltagEkfSlam apriltag_ekf_slam;
//
//   tf2::Transform robot_tf, robot_tf_new;
//
//   robot_tf.setOrigin(tf2::Vector3(2, 2, 0));
//   robot_tf.setRotation(tf2::Quaternion(0, 0, 0, 1));
//
//   robot_tf_new.setOrigin(tf2::Vector3(4, 2, 0));
//   robot_tf_new.setRotation(tf2::Quaternion(0, 0, 0.707106, 0.707106));
//
//   state_estimation::OdomInput u =
//   apriltag_ekf_slam.GetOdomInput_(&robot_tf, &robot_tf_new);
//
//   EXPECT_NEAR(u.trans_x, 2, 0.001);
//   EXPECT_NEAR(u.trans_y, 0, 0.001);
//   EXPECT_NEAR(u.rot, 1.57079, 0.001);
// }
//
// TEST(EkfSetup, GetMotionCov){
//   TestApriltagEkfSlam apriltag_ekf_slam;
//
//   state_estimation::OdomInput u;
//   u.trans_x = 2;
//   u.trans_y = 3;
//   u.rot = 1.570796;
//   Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
//
//   apriltag_ekf_slam.SetMotonModelParams_(0, 0, 0.2, 0.2, 0.2, 0.2);
//
//   apriltag_ekf_slam.GetMotionCov_(&u, &R);
//
//   EXPECT_NEAR(R(0, 0), 1.0352694, 0.001);
//   EXPECT_NEAR(R(0, 1), 0, 0.001);
//   EXPECT_NEAR(R(0, 2), 0, 0.001);
//   EXPECT_NEAR(R(1, 0), 0, 0.001);
//   EXPECT_NEAR(R(1, 1), 1.0352694, 0.001);
//   EXPECT_NEAR(R(1, 2), 0, 0.001);
//   EXPECT_NEAR(R(2, 0), 0, 0.001);
//   EXPECT_NEAR(R(2, 1), 0, 0.001);
//   EXPECT_NEAR(R(2, 2), 1.0352694, 0.001);
// }
//
// TEST(EkfSetup, GetMotionModelJacobian){
//   TestApriltagEkfSlam apriltag_ekf_slam;
//
//   state_estimation::OdomInput u;
//   u.trans_x = 2;
//   u.trans_y = 3;
//   u.rot = 1.570796;
//   Eigen::Matrix3d G = Eigen::Matrix3d::Zero();
//
//   state_estimation::StateBelief* state_belief = apriltag_ekf_slam.GetStateBelief_();
//
//   Eigen::VectorBlock<Eigen::VectorXd> mu_x = state_belief->mu_x();
//   mu_x(0) = 1;
//   mu_x(1) = 2;
//   mu_x(2) = 0.7853981634;
//
//   apriltag_ekf_slam.GetMotionModelJacobian_(&u, &G);
//
//   EXPECT_NEAR(G(0, 0), 1, 0.001);
//   EXPECT_NEAR(G(0, 1), 0, 0.001);
//   EXPECT_NEAR(G(0, 2), -3.535533906, 0.001);
//   EXPECT_NEAR(G(1, 0), 0, 0.001);
//   EXPECT_NEAR(G(1, 1), 1, 0.001);
//   EXPECT_NEAR(G(1, 2), -0.7071067812, 0.001);
//   EXPECT_NEAR(G(2, 0), 0, 0.001);
//   EXPECT_NEAR(G(2, 1), 0, 0.001);
//   EXPECT_NEAR(G(2, 2), 1, 0.001);
// }
//
// TEST(EkfSetup, UpdatePredictedMu){
//   TestApriltagEkfSlam apriltag_ekf_slam;
//
//   state_estimation::StateBelief* state_belief = apriltag_ekf_slam.GetStateBelief_();
//   Eigen::VectorBlock<Eigen::VectorXd> mu_x = state_belief->mu_x();
//
//   state_estimation::OdomInput u;
//   u.trans_x = 2;
//   u.trans_y = 3;
//   u.rot = 1.570796;
//
//   mu_x(0) = 4;
//   mu_x(1) = 2;
//   mu_x(2) = 1.570796;
//
//   apriltag_ekf_slam.UpdatePredictedMu_(&u);
//
//   EXPECT_NEAR(mu_x(0), 1, 0.001);
//   EXPECT_NEAR(mu_x(1), 4, 0.001);
//   EXPECT_NEAR(mu_x(2), 3.1415926, 0.001);
// }
//
// TEST(EkfSetup, GetTagMu){
//   TestApriltagEkfSlam apriltag_ekf_slam;
//
//   state_estimation::StateBelief* state_belief = apriltag_ekf_slam.GetStateBelief_();
//   Eigen::VectorBlock<Eigen::VectorXd> mu_x = state_belief->mu_x();
//
//   mu_x(0) = 2;
//   mu_x(1) = 1;
//   mu_x(2) = 0.785398163;
//
//   state_estimation::SensorMeasurement z;
//   z.tag_id = 100;
//   z.z.x() = 1;
//   z.z.y() = 1;
//   z.z.z() = 0;
//
//   Eigen::Vector3d tag_mu = apriltag_ekf_slam.GetTagMu_(&z);
//
//   EXPECT_NEAR(tag_mu(0), 2, 0.001);
//   EXPECT_NEAR(tag_mu(1), 2.4142135, 0.001);
//   EXPECT_NEAR(tag_mu(2), 0, 0.001);
// }

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_apriltag_ekf_slam");
  ros::NodeHandle nh;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
