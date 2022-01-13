#pragma once

/////// C++ Standard Template Library ///////
/////////////////////////////////////////////
#include <map>
#include <vector>
#include <cstdlib>
/////////////////////////////////////////////

/////////// ROS Standard Libraries ///////////
/////////////////////////////////////////////
#include <ros/ros.h>
/////////////////////////////////////////////

/////// Eigen (Linear Algebra Library) ///////
/////////////////////////////////////////////
#include <Eigen/Dense>
/////////////////////////////////////////////

using VectorBlock = Eigen::VectorBlock<Eigen::VectorXd>;
using MatrixBlock = Eigen::Block<Eigen::MatrixXd>;

namespace state_estimation{

  //! @brief It is a container class that is used to store belief
  //! about state of the robot and its environment as a gaussian
  //! density function.
  //!
  //! - It is assumed robot is operating in a planer environment so,
  //!   so its state (i.e. configuration) can be defined using three
  //!   variables (i.e. **x**, **y**, **yaw**)
  //! - Environment state is assumed to be defined by set of tag's
  //!   (with unique id) position (i.e. **x**, **y**, **z**)
  //! - _mu is the Expectation vector and _sigma is Covariance matrix
  //!   defining the StateBelief gaussian distribution
  struct StateBelief{
  public:
    StateBelief();
    //! @brief Adds new tag
    void NewTag(int tag_id, const Eigen::Vector3d& mu_tag);
    bool IsTracked(int tag_id) const;
    //! @return dimension of \f$\_mu\f$
    int GetSize() const;
    const std::vector<int>* GetTags() const;

    //! @return \f$\_ mu\f$
    VectorBlock mu();

    //! @return \f$X\f$
    VectorBlock mu_x();

    //! @return \f$M_i\f$ where, \f$i\f$ is the index corresponding to the tag_id
    VectorBlock mu_m(int tag_id);

    //! @return \f$\_sigma\f$
    MatrixBlock sigma();

    //! @return \f$\Sigma_{XX}\f$
    MatrixBlock sigma_xx();

    //! @return \f$\Sigma_{XM}\f$
    MatrixBlock sigma_xm();

    //! @return \f$\Sigma_{XM_i}\f$ where, \f$i\f$ is the index corresponding to the tag_id
    MatrixBlock sigma_xm(int tag_id);

    //! @return \f$\Sigma_{MX}\f$
    MatrixBlock sigma_mx();

    //! @return \f$\Sigma_{M_iX}\f$ where, \f$i\f$ is the index corresponding to the tag_id
    MatrixBlock sigma_mx(int tag_id);

    //! @return \f$\Sigma_{MM}\f$
    MatrixBlock sigma_mm();

    //! @return \f$\Sigma_{M_iM_j}\f$ where, \f$i\f$ and \f$j\f$ are the index corresponding to the tag_id_i and tag_id_j respectively
    MatrixBlock sigma_mm(int tag_id_i, int tag_id_j);

    //! @return \f$\left[\begin{array}{c}\Sigma_{XX}&\Sigma_{MX}\end{array}\right]\f$
    MatrixBlock sigma_star_x();

    //! @return \f$\left[\begin{array}{c} \Sigma_{XM_i} & \Sigma_{M_1M_i} & \vdots & \Sigma_{M_nM_i} \end{array}\right]\f$
    //! where, \f$i\f$ is the index corresponding to the tag_id
    MatrixBlock sigma_star_m(int tag_id);

    //! @return \f$\left[\begin{array}{cc}\Sigma_{XX}&\Sigma_{XM}\end{array}\right]\f$
    MatrixBlock sigma_x_star();

    //! @return \f$\left[\begin{array}{cccc} \Sigma_{M_iX} & \Sigma_{M_iM_1} & \ldots & \Sigma_{M_iM_n} \end{array}\right]\f$
    //! where, \f$i\f$ is the index corresponding to the tag_id
    MatrixBlock sigma_m_star(int tag_id);
  private:

    //! @brief Number of tags being tracked (i.e. \f$n\f$)
    int _no_tags;

    //! @brief Expectation vector of StateBelief gaussian distribution
    //!
    //! \f[ \_mu = \left[ \begin{array}{c} X & M \end{array} \right] \f]
    //! where, \f[ X =  \left[ \begin{array}{c} x & y & yaw \end{array} \right]  \in \Re^3 \f]
    //! and, \f[ M =  \left[ \begin{array}{c} M1 & M2 & \vdots & M_n \end{array} \right] \f]
    //! where, \f[ M_i =  \left[ \begin{array}{c} m_x & m_y & m_z \end{array} \right] \in \Re^3  \hspace{5mm} \forall i \in [1,2, \ldots, n] \f]
    //! and, \f$n\f$ indicate number of tags being tracked.
    Eigen::VectorXd _mu;

    //! @brief Covariance matrix of StateBelief gaussian distribution
    //!
    //! \f[ \_sigma = \left[ \begin{array}{cc} \Sigma_{XX} & \Sigma_{XM} & \Sigma_{MX} & \Sigma_{MM} \end{array} \right] \f]
    //! where, \f[ \Sigma_{XX} = \left[ \begin{array}{ccc} \sigma_{xx} & \sigma_{xy} & \sigma_{xyaw} & \sigma_{yx} & \sigma_{yy} & \sigma_{yyaw} & \sigma_{yawx} & \sigma_{yawy} & \sigma_{yawyaw} \end{array} \right] \in \Re^{3 \times 3} \f]
    //! and, \f[ \Sigma_{XM} = \left[ \begin{array}{cccc} \Sigma_{XM_{1}} & \Sigma_{XM_{2}} & \ldots & \Sigma_{XM_{n}}  \end{array} \right] \f]
    //! where, \f[ \Sigma_{XM_{i}} = \left[ \begin{array}{ccc} \sigma_{xm_{x}} & \sigma_{xm_{y}} & \sigma_{xm_{z}} & \sigma_{ym_{x}} & \sigma_{ym_{y}} & \sigma_{ym_{z}} & \sigma_{yawm_{x}} & \sigma_{yawm_{y}} & \sigma_{yawm_{z}} \end{array} \right] \Re^{3 \times 3}  \hspace{5mm} \forall i \in [1,2, \ldots, n] \f]
    //! and, \f[ \Sigma_{MX} = \left[ \begin{array}{c} \Sigma_{M_{1}X} & \Sigma_{M_{2}X} & \vdots & \Sigma_{M_{n}X}  \end{array} \right] \f]
    //! where, \f[ \Sigma_{M_{i}X} = \left[ \begin{array}{ccc} \sigma_{m_{x}x} & \sigma_{m_{x}y} & \sigma_{m_{x}yaw} & \sigma_{m_{y}x} & \sigma_{m_{y}y} & \sigma_{m_{y}yaw} & \sigma_{m_{z}x} & \sigma_{m_{z}y} & \sigma_{m_{z}yaw} \end{array} \right] \Re^{3 \times 3}  \hspace{5mm} \forall i \in [1,2, \ldots, n] \f]
    //! and, \f[ \Sigma_{MM} = \left[ \begin{array}{ccc} \Sigma_{M_{1}M_{1}} & \ldots & \Sigma_{M_{1}M_{n}} & \vdots & \ddots & \vdots & \Sigma_{M_{n}M_{1}} & \ldots &  \Sigma_{M_{n}M_{n}} \end{array} \right] \f]
    //! where, \f[ \Sigma_{M_{i}M_{j}} = \left[ \begin{array}{ccc} \sigma_{m_{x}m_{x}} & \sigma_{m_{x}m_{y}} & \sigma_{m_{x}m_{z}} & \sigma_{m_{y}m_{x}} & \sigma_{m_{y}m_{y}} & \sigma_{m_{y}m_{z}} & \sigma_{m_{z}m_{x}} & \sigma_{m_{z}m_{y}} & \sigma_{m_{z}m_{z}} \end{array} \right] \Re^{3 \times 3}  \hspace{5mm} \forall i,j \in [1,2, \ldots, n] \f]
    Eigen::MatrixXd _sigma;

    //! @brief Acts as a map between **tag_id** and its **index** as stored
    //!
    //! <tag_id, index>
    std::map<int, int> _data_association;

    //! @brief Acts as a map between tag **index** as stored and **tag_id**
    //!
    //! First element has index 1 and so on.
    std::vector<int> _tags;
  }; //   struct StateBelief
} // namespace state_estimation
