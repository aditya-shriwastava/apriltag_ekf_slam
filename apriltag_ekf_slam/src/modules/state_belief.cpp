#include "apriltag_ekf_slam/state_belief.h"

namespace state_estimation{
  StateBelief::StateBelief()
  : _no_tags{0}{
    this->_mu = Eigen::VectorXd::Zero(3);
    this->_sigma = Eigen::MatrixXd::Zero(3, 3);
  } // StateBelief::StateBelief

  void StateBelief::NewTag(int tag_id, const Eigen::Vector3d& mu_tag){
    if( this->IsTracked(tag_id) ){
      ROS_ERROR("This is not a new tag!!");
      std::exit(-1);
    }
    int size = 3 + (3 * this->_no_tags);
    int new_size = size + 3;
    this->_mu.conservativeResize(new_size);
    this->_sigma.conservativeResize(new_size, new_size);
    this->_no_tags++;
    this->_data_association.emplace(tag_id, this->_no_tags);
    this->_tags.emplace_back(tag_id);
    this->mu_m(tag_id) = mu_tag;
    this->sigma_mm(tag_id, tag_id) << 10000, 0, 0,
                                      0, 10000, 0,
                                      0, 0, 10000;
    this->_sigma.block(0, size, size, 3) = Eigen::MatrixXd::Zero(size, 3);
    this->_sigma.block(size, 0, 3, size) = Eigen::MatrixXd::Zero(3, size);
  } // StateBelief::NewTag

  bool StateBelief::IsTracked(int tag_id) const{
    if(this->_data_association.count(tag_id) > 0){
      return true;
    }else{
      return false;
    }
  } // StateBelief::IsTracked

  int StateBelief::GetSize() const{
    int size = 3 + (3 * this->_no_tags);
    return size;
  } // StateBelief::GetSize

  const std::vector<int>* StateBelief::GetTags() const{
    return &this->_tags;
  } // StateBelief::GetTags

  VectorBlock StateBelief::mu(){
    int size = 3 + (3 * this->_no_tags);
    return this->_mu.segment(0, size);
  } // StateBelief::mu

  VectorBlock StateBelief::mu_x(){
    return this->_mu.segment(0, 3);
  } // StateBelief::mu_x

  VectorBlock StateBelief::mu_m(int tag_id){
    if( !this->IsTracked(tag_id) ){
      ROS_ERROR("Tag %d does not exist!!", tag_id);
      std::exit(-1);
    }
    int pos = 3 + (3 * (this->_data_association[tag_id] - 1));
    return this->_mu.segment(pos, 3);
  } // StateBelief::mu_m

  MatrixBlock StateBelief::sigma(){
    int size = 3 + (3 * this->_no_tags);
    return this->_sigma.block(0, 0, size, size);
  } // StateBelief::sigma

  MatrixBlock StateBelief::sigma_xx(){
    return this->_sigma.block(0, 0, 3, 3);
  } // StateBelief::sigma_xx

  MatrixBlock StateBelief::sigma_xm(){
    int size = 3 * this->_no_tags;
    if( size == 0){
      ROS_WARN("Returned block is of size zero");
    }
    return this->_sigma.block(0, 3, 3, size);
  } // StateBelief::sigma_xm

  MatrixBlock StateBelief::sigma_xm(int tag_id){
    if( !this->IsTracked(tag_id) ){
      ROS_ERROR("Tag %d does not exist!!", tag_id);
      std::exit(-1);
    }
    int pos = 3 + (3 * (this->_data_association[tag_id] - 1));
    return this->_sigma.block(0, pos, 3, 3);
  } // StateBelief::sigma_xm

  MatrixBlock StateBelief::sigma_mx(){
    int size = 3 * this->_no_tags;
    if( size == 0){
      ROS_WARN("Returned block is of size zero");
    }
    return this->_sigma.block(3, 0, size, 3);
  } // StateBelief::sigma_mx

  MatrixBlock StateBelief::sigma_mx(int tag_id){
    if( !this->IsTracked(tag_id) ){
      ROS_ERROR("Tag %d does not exist!!", tag_id);
      std::exit(-1);
    }
    int pos = 3 + (3 * (this->_data_association[tag_id] - 1));
    return this->_sigma.block(pos, 0, 3, 3);
  } // StateBelief::sigma_mx

  MatrixBlock StateBelief::sigma_mm(){
    int size = 3 * this->_no_tags;
    if( size == 0){
      ROS_WARN("Returned block is of size zero");
    }
    return this->_sigma.block(3, 3, size, size);
  } // StateBelief::sigma_mm

  MatrixBlock StateBelief::sigma_mm(int tag_id_i, int tag_id_j){
    if( !(this->IsTracked(tag_id_i) && this->IsTracked(tag_id_j)) ){
      ROS_ERROR("Tag %d or %d does not exist!!", tag_id_i, tag_id_j);
      std::exit(-1);
    }
    int pos_i = 3 + (3 * (this->_data_association[tag_id_i] - 1));
    int pos_j = 3 + (3 * (this->_data_association[tag_id_j] - 1));
    return this->_sigma.block(pos_i, pos_j, 3, 3);
  } // StateBelief::sigma_mm

  MatrixBlock StateBelief::sigma_star_x(){
    int size = 3 + (3 * this->_no_tags);
    return this->_sigma.block(0, 0, size, 3);
  } // StateBelief::sigma_star_x

  MatrixBlock StateBelief::sigma_star_m(int tag_id){
    int pos = 3 + (3 * (this->_data_association[tag_id] - 1));
    int size = 3 + (3 * this->_no_tags);
    return this->_sigma.block(0, pos, size, 3);
  } // StateBelief::sigma_star_m

  MatrixBlock StateBelief::sigma_x_star(){
    int size = 3 + (3 * this->_no_tags);
    return this->_sigma.block(0, 0, 3, size);
  } // StateBelief::sigma_x_star

  MatrixBlock StateBelief::sigma_m_star(int tag_id){
    if( !this->IsTracked(tag_id) ){
      ROS_ERROR("Tag %d does not exist!!", tag_id);
      std::exit(-1);
    }
    int pos = 3 + (3 * (this->_data_association[tag_id] - 1));
    int size = 3 + (3 * this->_no_tags);
    return this->_sigma.block(pos, 0, 3, size);
  } // StateBelief::sigma_m_star
} // namespace state_estimation
