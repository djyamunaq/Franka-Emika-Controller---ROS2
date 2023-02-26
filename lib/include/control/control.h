#pragma once
#include "eigen3/Eigen/Dense"
#include "kinematic_model.h"

//! \brief Controller able to compute joint velocities for a given desired position
class Controller{
  public:
    //! \brief Controller of a 2-DoF robot described by a given model
    //! 
    //! \param model robot model
    Controller();
    Controller(RobotModel &model);

    //! \brief VJoint velocities for given joint position, desired cartesian position and feedforward term
    //!
    //! \return Dqd     joint velocities      
    //! \param  q       joint position
    //! \param  xd      desired cartesian position
    //! \param  Dxd_ff  joint velocity feedforward term
    Eigen::Vector<double, 7>   Dqd ( 
              const Eigen::Vector<double, 7> & q,
              const Eigen::Vector3d & Xd,
              const Eigen::Vector3d & DXd_ff,
              const Eigen::Matrix3d & Rd,
              const Eigen::Vector3d & DOd_ff
                          ); 
  private:
    RobotModel        model                                  ;    // Robot model
};