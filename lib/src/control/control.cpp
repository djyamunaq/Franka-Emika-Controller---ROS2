#include "control.h"
#include <iostream>

Controller::Controller() {}

Controller::Controller(RobotModel &rmIn):
  model  (rmIn)
{
}

Eigen::Vector<double, 7> Controller::Dqd ( 
          const Eigen::Vector<double, 7> & q,
          const Eigen::Vector3d & Xd,
          const Eigen::Vector3d & DXd_ff,
          const Eigen::Matrix3d & Rd,
          const Eigen::Vector3d & DOd_ff
                      )
{  
  //TODO Compute joint velocities able to track the desired cartesian position
  Eigen::Vector3d xOut;
  Eigen::Matrix3d ROut;
  Eigen::Matrix<double, 6, 7> JOut;

  this->model.FwdKin(xOut, ROut, JOut, q);

  const double kp {1e3};

  Eigen::AngleAxisd _R(Rd*(ROut.transpose()));
  
  Eigen::Vector3d dX = (Xd - xOut)*kp + DXd_ff;
  Eigen::Vector3d dO = _R.axis()*_R.angle()*kp + DOd_ff;

  Eigen::Vector<double, 6> v = {dX(0), dX(1), dX(2), dO(0), dO(1), dO(2)};

  /* Calculate pseudo-inverse for Jacobian Matrix */  
  Eigen::Matrix<double, 7, 6> J_pseudo_inv = JOut.completeOrthogonalDecomposition().pseudoInverse();

  /* Projection onto Null Space Matrix */
  Eigen::Matrix<double, 7, 7> P = Eigen::Matrix<double, 7, 7>::Identity() - J_pseudo_inv*JOut;
  /**/
  Eigen::Vector<double, 7> dwq0 = {0.009268810030863687, 0.006069607435986078, 0.009268810030863687, 0.009462246460398225, 0.009268810030863687, -0.001248594600026471, 0.009268810030863687};
  // double k0 = 1e1;

  // Eigen::Vector<double, 7> dq = J_pseudo_inv*v + P*dwq0;
  Eigen::Vector<double, 7> dq = J_pseudo_inv*v;
  
  return dq;
}

