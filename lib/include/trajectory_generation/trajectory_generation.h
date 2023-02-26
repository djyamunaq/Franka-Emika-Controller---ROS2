#pragma once
#include "eigen3/Eigen/Dense"
#include <iostream>

//! \brief Fifth degree polynomial with zero velocity and acceleration at initial and final time
//!
class Polynomial{
  public:
    //! \brief Default constructor
    Polynomial      ();

    //! \brief Polynomial for given initial and final position + time interval
    //!
    //! \param pi initial position
    //! \param pf final position
    //! \param Dt time interval
    Polynomial      (const double &piIn, const double &pfIn, const double & DtIn, const double & tiIn);

    //! \brief Update polynomial parameters
    //!
    //! \param pi initial position
    //! \param pf final position
    //! \param Dt time interval
    void      update(const double &piIn, const double &pfIn, const double & Dt, const double & tiIn);  

    //! \brief Compute position at given time
    //!
    //! \param t time
    const double p     (const double &t);

    //! \brief Compute velocity at given time
    //!
    //! \param t time
    const double dp    (const double &t);
  private: 
    std::array<double,6>  a     {   0   };    // Polynomial coefficients
    double                Dt    {   0   };    // Time interval
    double                pi    {   0   };    // Initial position
    double                pf    {   0   };    // Final position
    double                ti    {   0   };
};

//! \brief Point to point trajectory generator
class Point2Point{
  public:
    //! \brief Point to point trajectory generator with given initial and final positions + time interval
    //!
    //! \param X_i initial position (x,y, z, qx, qy, qz)
    //! \param X_f final   position (x,y, z, qx, qy, qz)
    //! \param Dt  time interval
    Point2Point(const Eigen::Vector<double, 3> & xi, const Eigen::Quaterniond quati, const Eigen::Vector<double, 3> & xf, const Eigen::Quaterniond quatf, const double & DtIn, const double & tiIn);
    
    Point2Point();

    //! \brief Compute position at given time
    //!
    //! \param time 
    Eigen::Vector<double, 3> X (const double & time) ;

    //! \brief Compute velocity at given time
    //!
    //! \param time
    Eigen::Vector<double, 3> dX(const double & time) ;

    Eigen::Quaterniond Q(const double & t);

    Eigen::Vector<double, 3> dQ(const double & t);

  private:
    Polynomial      polx                    ;   // Polynomial to x coordinates
    Polynomial      poly                    ;   // Polynomial to y coordinates    
    Polynomial      polz                    ;   // Polynomial to y coordinates    
    double          Dt          {   0   }   ;   // time interval
    double          ti          {   0   }   ;   // initial instant
    Eigen::Quaterniond qi;
    Eigen::Quaterniond qf;
};