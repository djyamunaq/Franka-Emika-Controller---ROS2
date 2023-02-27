#include "control.h"
#include <iostream>

Controller::Controller() {}

Controller::Controller(RobotModel &rmIn) : model  (rmIn) {
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

  const double kp {1e1};

  Eigen::AngleAxisd _R(Rd*(ROut.transpose()));
  
  Eigen::Vector3d dX = (Xd - xOut)*kp + DXd_ff;
  Eigen::Vector3d dO = _R.axis()*_R.angle()*kp + DOd_ff;

  Eigen::Vector<double, 6> v = {dX(0), dX(1), dX(2), dO(0), dO(1), dO(2)};

  /* Max and Min qi */
  double q1_min = -2.8973;
  double q1_max = 2.8973;
  double q2_min = -1.7628;
  double q2_max = 1.7628;
  double q3_min = -2.8973;
  double q3_max = 2.8973;
  double q4_min = -3.0718;
  double q4_max = -0.0698;
  double q5_min = -2.8973;
  double q5_max = 2.8973;
  double q6_min = -0.0175;
  double q6_max = 3.7525;
  double q7_min = -2.8973;
  double q7_max = 2.8973;
  Eigen::Vector<double, 7> qs_min = {q1_min, q2_min, q3_min, q4_min, q5_min, q6_min, q7_min};
  Eigen::Vector<double, 7> qs_max = {q1_max, q2_max, q3_max, q4_max, q5_max, q6_max, q7_max};

  Eigen::Vector<double, 7> q_vel_target;

  int n = 7;
  double beta = 1e1;
  for(int i=0; i<n; i++) {
    double q_avg = (qs_max[i] + qs_min[i])/2;

    double q_range = (qs_max[i] - qs_min[i]);

    q_vel_target[i] = beta*((q[i] - q_avg)/q_range);
  }
  
  // std::cout << q_vel_target << std::endl;

  /* Calculate pseudo-inverse for Jacobian Matrix */  
  Eigen::Matrix<double, 7, 6> J_pseudo_inv = JOut.completeOrthogonalDecomposition().pseudoInverse();

  /* Projection onto Null Space Matrix */
  Eigen::Matrix<double, 7, 7> P = Eigen::Matrix<double, 7, 7>::Identity() - J_pseudo_inv*JOut;
  Eigen::Vector<double, 7> z = q_vel_target;
  Eigen::Vector<double, 7> dq = J_pseudo_inv*v - P*z;
  // Eigen::Vector<double, 7> dq = J_pseudo_inv*v;

  // double dt = 1e-3;
  // Eigen::Vector<double, 7> q_new = q + dq*dt;
  // if(q_new[0] > qs_max[0] || q_new[0] < qs_min[0]) {
  //   std::cout << "q1" << q_new[0] << std::endl;
  //   exit(1);
  // } else if(q_new[1] > qs_max[1] || q_new[1] < qs_min[1]) {
  //   std::cout << "q2" << q_new[1] << std::endl;
  //   exit(1);
  // } else if(q_new[2] > qs_max[2] || q_new[2] < qs_min[2]) {
  //   std::cout << "q3" << q_new[2] << std::endl;
  //   exit(1);
  // } else if(q_new[3] > qs_max[3] || q_new[3] < qs_min[3]) {
  //   std::cout << "q4" << q_new[3] << std::endl;
  //   exit(1);
  // } else if(q_new[4] > qs_max[4] || q_new[4] < qs_min[4]) {
  //   std::cout << "q5" << q_new[4] << std::endl;
  //   exit(1);
  // } else if(q_new[5] > qs_max[5] || q_new[5] < qs_min[5]) {
  //   std::cout << "q6" << q_new[5] << std::endl;
  //   exit(1);
  // } else if(q_new[6] > qs_max[6] || q_new[6] < qs_min[6]) {
  //   std::cout << "q7" << q_new[6] << std::endl;
  //   exit(1);
  // }

  return dq;
}

