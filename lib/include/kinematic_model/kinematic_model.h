#pragma once
#include "eigen3/Eigen/Dense"
#include <cmath>
#include <vector>
#include <iostream>

//! \brief Kinematic model of a 2-DoF robot (Forward and differential)
class RobotModel{
  public:
    RobotModel();
    RobotModel(std::vector<double> a, std::vector<double> d, std::vector<double> alpha);

    //! \brief Compute position and jacobian matrix for given joint position
    //!
    //! \param x Output cartesian position
    //! \param J Output jacobian matrix
    //! \param q joint position
    void FwdKin(Eigen::Vector3d &xOut, Eigen::Matrix3d &ROut, Eigen::Matrix<double, 6, 7> &JOut, const Eigen::Vector<double, 7> & qIn);
  private:
    std::vector<double> a;   // length 1
    std::vector<double> d;   // length 1
    std::vector<double> alpha;   // length 1
};