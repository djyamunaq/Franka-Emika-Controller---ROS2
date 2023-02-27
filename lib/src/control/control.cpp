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
  Eigen::Vector<double, 7> q_min = {q1_min, q2_min, q3_min, q4_min, q5_min, q6_min, q7_min};
  Eigen::Vector<double, 7> q_max = {q1_max, q2_max, q3_max, q4_max, q5_max, q6_max, q7_max};

  /* Calculate pseudo-inverse for Jacobian Matrix */  
  Eigen::Matrix<double, 7, 6> J_pseudo_inv = JOut.completeOrthogonalDecomposition().pseudoInverse();

  /* Projection onto Null Space Matrix */
  Eigen::Matrix<double, 7, 7> P = Eigen::Matrix<double, 7, 7>::Identity() - J_pseudo_inv*JOut;

  /* Median position joints */
  Eigen::Vector<double, 7> q_med = {0, 0, 0, -1.5, 0, 2, 0};
  /*  */
  Eigen::Vector<double, 7> Dq = q_med - q;

  // Eigen::Vector<double, 7> dq = J_pseudo_inv*v - P*t1;
  Eigen::Vector<double, 7> dq = J_pseudo_inv*v;

  /* Range limit */
    /* Define alpha functions */
  auto alpha1 = [](double x) {
    double a = 2*2.8973;
    double b = -0.5;
    double c = 0;
    double k = ((x-c)/a) - b;
    return log(k/(1 - k));
  };
  auto alpha2 = [](double x) {
    double a = 2*1.7628;
    double b = -0.5;
    double c = 0;
    double k = ((x-c)/a) - b;
    return log(k/(1 - k));
  };
  auto alpha3 = [](double x) {
    double a = 2*2.8973;
    double b = -0.5;
    double c = 0;
    double k = ((x-c)/a) - b;
    return log(k/(1 - k));
  };
  auto alpha4 = [](double x) {
    double a = (-0.0698 + 3.0718);
    double b = -0.5;
    double c = - (-0.0698 + 3.0718)/2 - 0.0698;
    double k = ((x-c)/a) - b;
    return log(k/(1 - k));
  };
  auto alpha5 = [](double x) {
    double a = 2*2.8973;
    double b = -0.5;
    double c = 0;
    double k = ((x-c)/a) - b;
    return log(k/(1 - k));
  };
  auto alpha6 = [](double x) {
    double a = (3.7525 + 0.0175);
    double b = -0.5;
    double c = - (3.7525 + 0.0175)/2 + 3.7525;
    double k = ((x-c)/a) - b;
    return log(k/(1 - k));
  };
  auto alpha7 = [](double x) {
    double a = 2*2.8973;
    double b = -0.5;
    double c = 0;
    double k = ((x-c)/a) - b;
    return log(k/(1 - k));
  };
    /* Define beta functions */
  auto beta1 = [](double x) {
    return 2*2.8973*((1/(1 + exp(-x))) - 0.5);
  };
  auto beta2 = [](double x) {
    return 2*1.7628*((1/(1 + exp(-x))) - 0.5);
  };
  auto beta3 = [](double x) {
    return 2*2.8973*((1/(1 + exp(-x))) - 0.5);
  };
  auto beta4 = [](double x) {
    return (-0.0698 + 3.0718)*((1/(1 + exp(-x))) - 0.5) - (-0.0698 + 3.0718)/2 - 0.0698;
  };
  auto beta5 = [](double x) {
    return 2*2.8973*((1/(1 + exp(-x))) - 0.5);
  };
  auto beta6 = [](double x) {
    return (3.7525 + 0.0175)*((1/(1 + exp(-x))) - 0.5) - (3.7525 + 0.0175)/2 + 3.7525;
  };
  auto beta7 = [](double x) {
    double sigma = (1/(1 + exp(-x)));
    return 2*2.8973*(sigma*(1- sigma));
  };
  /* Define dbeta functions */
  auto dbeta1 = [](double x) {
    double sigma = (1/(1 + exp(-x)));
    return 2*2.8973*(sigma*(1- sigma));
  };
  auto dbeta2 = [](double x) {
    double sigma = (1/(1 + exp(-x)));
    return 2*1.7628*(sigma*(1- sigma));
  };
  auto dbeta3 = [](double x) {
    double sigma = (1/(1 + exp(-x)));
    return 2*2.8973*(sigma*(1- sigma));
  };
  auto dbeta4 = [](double x) {
    double sigma = (1/(1 + exp(-x)));
    return (-0.0698 + 3.0718)*(sigma*(1 - sigma));
  };
  auto dbeta5 = [](double x) {
    double sigma = (1/(1 + exp(-x)));
    return 2*2.8973*(sigma*(1- sigma));
  };
  auto dbeta6 = [](double x) {
    double sigma = (1/(1 + exp(-x)));
    return (3.7525 + 0.0175)*(sigma*(1- sigma));
  };
  auto dbeta7 = [](double x) {
    double sigma = (1/(1 + exp(-x)));
    return 2*2.8973*(sigma*(1- sigma));
  };

  /* Find zi values */
  double z1 = alpha1(q[0]);
  double z2 = alpha2(q[1]);
  double z3 = alpha3(q[2]);
  double z4 = alpha4(q[3]);
  double z5 = alpha5(q[4]);
  double z6 = alpha6(q[5]);
  double z7 = alpha7(q[6]);
  Eigen::Vector<double, 7> z = {z1, z2, z3, z4, z5, z6, z7};

  /* find matrix dBeta */
  Eigen::Matrix<double, 7, 7> dBeta;
  dBeta <<  dbeta1(z1), 0, 0, 0, 0, 0, 0,
            0, dbeta2(z2), 0, 0, 0, 0, 0,
            0, 0, dbeta3(z3), 0, 0, 0, 0,
            0, 0, 0, dbeta4(z4), 0, 0, 0,
            0, 0, 0, 0, dbeta5(z5), 0, 0,
            0, 0, 0, 0, 0, dbeta6(z6), 0,
            0, 0, 0, 0, 0, 0, dbeta7(z7);

  Eigen::Matrix<double, 7, 7> dBeta_inv = dBeta.inverse();
  Eigen::Vector<double, 7> dz = dBeta_inv*dq;

  z += dz;

  Eigen::Vector<double, 7> q_new;
  q_new[0] = beta1(z[0]);
  q_new[1] = beta2(z[1]);
  q_new[2] = beta3(z[2]);
  q_new[3] = beta4(z[3]);
  q_new[4] = beta5(z[4]);
  q_new[5] = beta6(z[5]);
  q_new[6] = beta7(z[6]);

  // std::cout << (dq - (q_new - q)).transpose() << std::endl; 
  dq = q_new - q;

  q_new = q + dq;

  if(q_new[0] > q_max[0] || q_new[0] < q_min[0]) {
    std::cout << "q0" << q_new[0] << std::endl;
    exit(1);
  } else if(q_new[1] > q_max[1] || q_new[1] < q_min[1]) {
    std::cout << "q1" << q_new[1] << std::endl;
    exit(1);
  } else if(q_new[2] > q_max[2] || q_new[2] < q_min[2]) {
    std::cout << "q2" << q_new[2] << std::endl;
    exit(1);
  } else if(q_new[3] > q_max[3] || q_new[3] < q_min[3]) {
    std::cout << "q3" << q_new[3] << std::endl;
    exit(1);
  } else if(q_new[4] > q_max[4] || q_new[4] < q_min[4]) {
    std::cout << "q4" << q_new[4] << std::endl;
    exit(1);
  } else if(q_new[5] > q_max[5] || q_new[5] < q_min[5]) {
    std::cout << "q5" << q_new[5] << std::endl;
    exit(1);
  } else if(q_new[6] > q_max[6] || q_new[6] < q_min[6]) {
    std::cout << "q6" << q_new[6] << std::endl;
    exit(1);
  }

  return dq;
}

