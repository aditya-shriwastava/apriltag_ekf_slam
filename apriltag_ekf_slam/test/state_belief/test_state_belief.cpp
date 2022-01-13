#include <gtest/gtest.h>
#include <ros/ros.h>
#include "apriltag_ekf_slam/state_belief.h"

TEST(NoTag, IsTracked){
  state_estimation::StateBelief state_belief;
  EXPECT_EQ( state_belief.IsTracked(1) ,false);
  EXPECT_EQ( state_belief.IsTracked(0) ,false);
  EXPECT_EQ( state_belief.IsTracked(-6) ,false);
  EXPECT_EQ( state_belief.IsTracked(10) ,false);
  EXPECT_EQ( state_belief.IsTracked(-1) ,false);
}

TEST(NoTag, GetSize){
  state_estimation::StateBelief state_belief;
  EXPECT_EQ( state_belief.GetSize() ,3);
}

TEST(NoTag, GetTags){
  state_estimation::StateBelief state_belief;
  EXPECT_EQ( state_belief.GetTags()->size() ,0);
}

TEST(NoTag, mu){
  state_estimation::StateBelief state_belief;
  Eigen::VectorBlock<Eigen::VectorXd> mu = state_belief.mu();
  EXPECT_EQ( mu(0) ,0);
  EXPECT_EQ( mu(1) ,0);
  EXPECT_EQ( mu(2) ,0);

  EXPECT_EQ( mu.size() ,3);

  mu = Eigen::Vector3d::Constant(5);

  Eigen::VectorBlock<Eigen::VectorXd> mu_new = state_belief.mu();
  EXPECT_EQ( mu_new(0) ,5);
  EXPECT_EQ( mu_new(1) ,5);
  EXPECT_EQ( mu_new(2) ,5);
}

TEST(NoTag, mu_x){
  state_estimation::StateBelief state_belief;
  Eigen::VectorBlock<Eigen::VectorXd> mu_x = state_belief.mu_x();
  mu_x = Eigen::Vector3d::Constant(5);
  EXPECT_EQ( mu_x(0) ,5);
  EXPECT_EQ( mu_x(1) ,5);
  EXPECT_EQ( mu_x(2) ,5);

  EXPECT_EQ( mu_x.size() ,3);
}

TEST(NoTag, sigma){
  state_estimation::StateBelief state_belief;
  Eigen::Block<Eigen::MatrixXd> sigma = state_belief.sigma();

  EXPECT_EQ( sigma(0,0) ,0);
  EXPECT_EQ( sigma(0,1) ,0);
  EXPECT_EQ( sigma(0,2) ,0);
  EXPECT_EQ( sigma(1,0) ,0);
  EXPECT_EQ( sigma(1,1) ,0);
  EXPECT_EQ( sigma(1,2) ,0);
  EXPECT_EQ( sigma(2,0) ,0);
  EXPECT_EQ( sigma(2,1) ,0);
  EXPECT_EQ( sigma(2,2) ,0);

  EXPECT_EQ( sigma.rows() ,3);
  EXPECT_EQ( sigma.cols() ,3);

  Eigen::Block<Eigen::MatrixXd> sigma_new = state_belief.sigma();

  sigma_new = Eigen::Matrix3d::Constant(7);

  EXPECT_EQ( sigma_new(0,0) ,7);
  EXPECT_EQ( sigma_new(0,1) ,7);
  EXPECT_EQ( sigma_new(0,2) ,7);
  EXPECT_EQ( sigma_new(1,0) ,7);
  EXPECT_EQ( sigma_new(1,1) ,7);
  EXPECT_EQ( sigma_new(1,2) ,7);
  EXPECT_EQ( sigma_new(2,0) ,7);
  EXPECT_EQ( sigma_new(2,1) ,7);
  EXPECT_EQ( sigma_new(2,2) ,7);
}

TEST(NoTag, sigma_xx){
  state_estimation::StateBelief state_belief;
  Eigen::Block<Eigen::MatrixXd> sigma_xx = state_belief.sigma_xx();

  EXPECT_EQ( sigma_xx(0,0) ,0);
  EXPECT_EQ( sigma_xx(0,1) ,0);
  EXPECT_EQ( sigma_xx(0,2) ,0);
  EXPECT_EQ( sigma_xx(1,0) ,0);
  EXPECT_EQ( sigma_xx(1,1) ,0);
  EXPECT_EQ( sigma_xx(1,2) ,0);
  EXPECT_EQ( sigma_xx(2,0) ,0);
  EXPECT_EQ( sigma_xx(2,1) ,0);
  EXPECT_EQ( sigma_xx(2,2) ,0);

  EXPECT_EQ( sigma_xx.rows() ,3);
  EXPECT_EQ( sigma_xx.cols() ,3);
}

TEST(NoTag, sigma_star_x){
  state_estimation::StateBelief state_belief;
  Eigen::Block<Eigen::MatrixXd> sigma_star_x = state_belief.sigma_star_x();

  EXPECT_EQ( sigma_star_x(0,0) ,0);
  EXPECT_EQ( sigma_star_x(0,1) ,0);
  EXPECT_EQ( sigma_star_x(0,2) ,0);
  EXPECT_EQ( sigma_star_x(1,0) ,0);
  EXPECT_EQ( sigma_star_x(1,1) ,0);
  EXPECT_EQ( sigma_star_x(1,2) ,0);
  EXPECT_EQ( sigma_star_x(2,0) ,0);
  EXPECT_EQ( sigma_star_x(2,1) ,0);
  EXPECT_EQ( sigma_star_x(2,2) ,0);

  EXPECT_EQ( sigma_star_x.rows() ,3);
  EXPECT_EQ( sigma_star_x.cols() ,3);
}

TEST(NoTag, sigma_x_star){
  state_estimation::StateBelief state_belief;
  Eigen::Block<Eigen::MatrixXd> sigma_x_star = state_belief.sigma_x_star();

  EXPECT_EQ( sigma_x_star(0,0) ,0);
  EXPECT_EQ( sigma_x_star(0,1) ,0);
  EXPECT_EQ( sigma_x_star(0,2) ,0);
  EXPECT_EQ( sigma_x_star(1,0) ,0);
  EXPECT_EQ( sigma_x_star(1,1) ,0);
  EXPECT_EQ( sigma_x_star(1,2) ,0);
  EXPECT_EQ( sigma_x_star(2,0) ,0);
  EXPECT_EQ( sigma_x_star(2,1) ,0);
  EXPECT_EQ( sigma_x_star(2,2) ,0);

  EXPECT_EQ( sigma_x_star.rows() ,3);
  EXPECT_EQ( sigma_x_star.cols() ,3);
}

class OneTagTest : public ::testing::Test{
 protected:
   state_estimation::StateBelief state_belief;
   virtual void SetUp(){
     Eigen::Vector3d mu_tag;
     mu_tag(0) = 1;
     mu_tag(1) = 2;
     mu_tag(2) = 3;
     state_belief.NewTag(7, mu_tag);
   }
};

TEST_F(OneTagTest, IsTracked){
  EXPECT_EQ( state_belief.IsTracked(1) ,false);
  EXPECT_EQ( state_belief.IsTracked(7) ,true);
}

TEST_F(OneTagTest, GetSize){
  EXPECT_EQ( state_belief.GetSize() ,6);
}

TEST_F(OneTagTest, GetTags){
  const std::vector<int>* tags = state_belief.GetTags();
  EXPECT_EQ( tags->size() ,1);
  EXPECT_EQ( tags->at(0) ,7);
}

TEST_F(OneTagTest, mu){
  Eigen::VectorBlock<Eigen::VectorXd> mu = state_belief.mu();
  EXPECT_EQ( mu(0) ,0);
  EXPECT_EQ( mu(1) ,0);
  EXPECT_EQ( mu(2) ,0);
  EXPECT_EQ( mu(3) ,1);
  EXPECT_EQ( mu(4) ,2);
  EXPECT_EQ( mu(5) ,3);

  EXPECT_EQ( mu.size() ,6);
}

TEST_F(OneTagTest, mu_x){
  Eigen::VectorBlock<Eigen::VectorXd> mu_x = state_belief.mu_x();
  mu_x = Eigen::Vector3d::Constant(5);
  EXPECT_EQ( mu_x(0) ,5);
  EXPECT_EQ( mu_x(1) ,5);
  EXPECT_EQ( mu_x(2) ,5);

  EXPECT_EQ( mu_x.size() ,3);
}

TEST_F(OneTagTest, sigma){
  Eigen::Block<Eigen::MatrixXd> sigma = state_belief.sigma();

  EXPECT_EQ( sigma.rows() ,6);
  EXPECT_EQ( sigma.cols() ,6);

  for(int i=0; i<6; i++){
    for(int j=0; j<6; j++){
      if( (i==3) && (j==3) ){
        EXPECT_EQ( sigma(i,j) ,10000);
      }else if( (i==4) && (j==4) ){
        EXPECT_EQ( sigma(i,j) ,10000);
      }else if( (i==5) && (j==5) ){
        EXPECT_EQ( sigma(i,j) ,10000);
      }else{
        EXPECT_EQ( sigma(i,j) ,0);
      }
    }
  }
}

TEST_F(OneTagTest, sigma_xx){
  Eigen::Block<Eigen::MatrixXd> sigma_xx = state_belief.sigma_xx();

  EXPECT_EQ( sigma_xx.rows() ,3);
  EXPECT_EQ( sigma_xx.cols() ,3);

  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      EXPECT_EQ( sigma_xx(i,j) ,0);
    }
  }
}

TEST_F(OneTagTest, sigma_xm){
  Eigen::Block<Eigen::MatrixXd> sigma_xm = state_belief.sigma_xm();

  EXPECT_EQ( sigma_xm.rows() ,3);
  EXPECT_EQ( sigma_xm.cols() ,3);

  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      EXPECT_EQ( sigma_xm(i,j) ,0);
    }
  }
}

TEST_F(OneTagTest, sigma_mx){
  Eigen::Block<Eigen::MatrixXd> sigma_mx = state_belief.sigma_mx();

  EXPECT_EQ( sigma_mx.rows() ,3);
  EXPECT_EQ( sigma_mx.cols() ,3);

  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      EXPECT_EQ( sigma_mx(i,j) ,0);
    }
  }
}

TEST_F(OneTagTest, sigma_xm_t){
  Eigen::Block<Eigen::MatrixXd> sigma_xm = state_belief.sigma_xm(7);

  EXPECT_EQ( sigma_xm.rows() ,3);
  EXPECT_EQ( sigma_xm.cols() ,3);

  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      EXPECT_EQ( sigma_xm(i,j) ,0);
    }
  }
}

TEST_F(OneTagTest, sigma_mx_t){
  Eigen::Block<Eigen::MatrixXd> sigma_mx = state_belief.sigma_mx(7);

  EXPECT_EQ( sigma_mx.rows() ,3);
  EXPECT_EQ( sigma_mx.cols() ,3);

  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      EXPECT_EQ( sigma_mx(i,j) ,0);
    }
  }
}

TEST_F(OneTagTest, sigma_mm){
  Eigen::Block<Eigen::MatrixXd> sigma_mm = state_belief.sigma_mm();

  EXPECT_EQ( sigma_mm.rows() ,3);
  EXPECT_EQ( sigma_mm.cols() ,3);

  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      if(i == j){
        EXPECT_EQ( sigma_mm(i,j) ,10000);
      }else{
        EXPECT_EQ( sigma_mm(i,j) ,0);
      }
    }
  }
}

TEST_F(OneTagTest, sigma_mm_t){
  Eigen::Block<Eigen::MatrixXd> sigma_mm = state_belief.sigma_mm(7,7);

  EXPECT_EQ( sigma_mm.rows() ,3);
  EXPECT_EQ( sigma_mm.cols() ,3);

  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      if(i == j){
        EXPECT_EQ( sigma_mm(i,j) ,10000);
      }else{
        EXPECT_EQ( sigma_mm(i,j) ,0);
      }
    }
  }
}

TEST_F(OneTagTest, sigma_star_x){
  Eigen::Block<Eigen::MatrixXd> sigma_star_x = state_belief.sigma_star_x();

  EXPECT_EQ( sigma_star_x.rows() ,6);
  EXPECT_EQ( sigma_star_x.cols() ,3);

  for(int i=0; i<6; i++){
    for(int j=0; j<3; j++){
      EXPECT_EQ( sigma_star_x(i,j) ,0);
    }
  }
}

TEST_F(OneTagTest, sigma_star_m){
  Eigen::Block<Eigen::MatrixXd> sigma_star_m = state_belief.sigma_star_m(7);

  EXPECT_EQ( sigma_star_m.rows() ,6);
  EXPECT_EQ( sigma_star_m.cols() ,3);

  for(int i=0; i<6; i++){
    for(int j=0; j<3; j++){
      if( (i==3) && (j==0) ){
        EXPECT_EQ( sigma_star_m(i,j) ,10000);
      }else if( (i==4) && (j==1) ){
        EXPECT_EQ( sigma_star_m(i,j) ,10000);
      }else if( (i==5) && (j==2) ){
        EXPECT_EQ( sigma_star_m(i,j) ,10000);
      }else{
        EXPECT_EQ( sigma_star_m(i,j) ,0);
      }
    }
  }

}

TEST_F(OneTagTest, sigma_x_star){
  Eigen::Block<Eigen::MatrixXd> sigma_x_star = state_belief.sigma_x_star();

  EXPECT_EQ( sigma_x_star.rows() ,3);
  EXPECT_EQ( sigma_x_star.cols() ,6);

  for(int i=0; i<3; i++){
    for(int j=0; j<6; j++){
      EXPECT_EQ( sigma_x_star(i,j) ,0);
    }
  }

}

TEST_F(OneTagTest, sigma_m_star){
  Eigen::Block<Eigen::MatrixXd> sigma_m_star = state_belief.sigma_m_star(7);

  EXPECT_EQ( sigma_m_star.rows() ,3);
  EXPECT_EQ( sigma_m_star.cols() ,6);

  for(int i=0; i<3; i++){
    for(int j=0; j<6; j++){
      if( (i==0) && (j==3) ){
        EXPECT_EQ( sigma_m_star(i,j) ,10000);
      }else if( (i==1) && (j==4) ){
        EXPECT_EQ( sigma_m_star(i,j) ,10000);
      }else if( (i==2) && (j==5) ){
        EXPECT_EQ( sigma_m_star(i,j) ,10000);
      }else{
        EXPECT_EQ( sigma_m_star(i,j) ,0);
      }
    }
  }

}

class TwoTagTest : public ::testing::Test{
 protected:
   state_estimation::StateBelief state_belief;
   virtual void SetUp(){
     Eigen::Vector3d mu_tag;
     mu_tag(0) = 1;
     mu_tag(1) = 2;
     mu_tag(2) = 3;
     state_belief.NewTag(7, mu_tag);

     mu_tag(0) = 4;
     mu_tag(1) = 5;
     mu_tag(2) = 6;
     state_belief.NewTag(3, mu_tag);
   }
};

TEST_F(TwoTagTest, sigma_mm_t){
  Eigen::Block<Eigen::MatrixXd> sigma_mm_3_7 = state_belief.sigma_mm(3,7);

  EXPECT_EQ( sigma_mm_3_7.rows() ,3);
  EXPECT_EQ( sigma_mm_3_7.cols() ,3);

  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      EXPECT_EQ( sigma_mm_3_7(i,j) ,0);
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_state_belief");
  ros::NodeHandle nh;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
