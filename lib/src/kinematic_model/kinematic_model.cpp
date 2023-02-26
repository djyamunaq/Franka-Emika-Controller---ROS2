#include "kinematic_model.h"

RobotModel::RobotModel() {}

RobotModel::RobotModel(std::vector<double> a, std::vector<double> d, std::vector<double> alpha) {
  this->a = a;
  this->d = d;
  this->alpha = alpha;
};

void RobotModel::FwdKin(Eigen::Vector3d &xOut, Eigen::Matrix3d &ROut, Eigen::Matrix<double, 6, 7> &JOut, const Eigen::Vector<double, 7> & qIn) {
  // TODO Implement the forward and differential kinematics
  
  Eigen::Matrix<double, 4, 4> Tcurr;
  std::vector<Eigen::Matrix<double, 4, 4>> Ts;
  Eigen::Matrix<double, 4, 4> T = Eigen::Matrix4d::Identity();

  /* Calculate Transformation Matrices -> (i-1)Ti */
  for(int i=0; i<7; i++) {
    Eigen::Matrix<double, 4, 4> Tcurr;
    Tcurr <<  cos(qIn(i)), -sin(qIn(i)), 0, a[i],
              sin(qIn(i))*cos(alpha[i]), cos(qIn(i))*cos(alpha[i]), -sin(alpha[i]), -d[i]*sin(alpha[i]),
              sin(qIn(i))*sin(alpha[i]), cos(qIn(i))*sin(alpha[i]), cos(alpha[i]), d[i]*cos(alpha[i]),
              0, 0, 0, 1;    

    Ts.push_back(Tcurr);
    T = T*Tcurr;
  }

  /* Get Rotation Matrix and Carthesian Pos for End-Effector */
  xOut = T.block<3, 1>(0, 3);
  ROut = T.block<3, 3>(0, 0);
  Eigen::Vector3d pe = Eigen::Vector3d(xOut); 

  /* Jacobian Matrix */
  JOut = Eigen::Matrix<double, 6, 7>::Identity();

  /* Calculate Jp1 */
  Eigen::Vector3d z0 = {0, 0, 1};
  Eigen::Vector3d p0 = {0, 0, 0};
  Eigen::Vector3d Jp1 = z0.cross(pe - p0);
  Eigen::Vector3d Jo1 = z0;

  for(int i=0; i<7; i++) {
    /* Temporary Transformation Matrix from frame 0 to i */
    Eigen::Matrix4d tempT = Eigen::Matrix4d::Identity();
    
    for(int j=0; j<i+1; j++) {
      tempT = tempT*Ts[j];
    }

    Eigen::Vector3d z = tempT.block<3, 1>(0, 2);
    Eigen::Vector3d p = tempT.block<3, 1>(0, 3);
    Eigen::Vector3d Jpi = z.cross(pe - p);
    Eigen::Vector3d Joi = z;

    JOut.col(i) <<  Jpi(0), 
                    Jpi(1), 
                    Jpi(2), 
                    Joi(0), 
                    Joi(1), 
                    Joi(2);
  }

  ROut = ROut.unaryExpr([](double elem) {
    double threshold = 1e-10;
    return elem = (abs(elem) < threshold) ? 0.0 : elem; 
  });

  xOut = xOut.unaryExpr([](double elem) {
    double threshold = 1e-10;
    return elem = (abs(elem) < threshold) ? 0.0 : elem; 
  });

  JOut = JOut.unaryExpr([](double elem) {
    double threshold = 1e-10;
    return elem = (abs(elem) < threshold) ? 0.0 : elem; 
  });

  // std::cout << "Jacobian:" << std::endl;
  // std::cout << JOut << std::endl;
  // exit(0);
}