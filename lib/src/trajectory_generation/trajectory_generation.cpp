#include "trajectory_generation.h"

Polynomial::Polynomial(){};

Polynomial::Polynomial(const double &piIn, const double &pfIn, const double & DtIn, const double & tiIn){
  //TODO initialize the object polynomial coefficients
  this->pi = piIn;
  this->pf = pfIn;
  this->ti = tiIn;
  this->Dt = DtIn;

  Eigen::Vector<double, 6> coef = Eigen::Vector<double, 6>::Zero();
  Eigen::Vector<double, 6> L = Eigen::Vector<double, 6>::Zero();
  L << this->pi, this->pf, 0, 0, 0, 0;
  Eigen::Matrix<double,6,6> A = Eigen::Matrix<double,6,6>::Zero(); 
  A <<  0, 0, 0, 0, 0, 1,
        pow(this->Dt, 5), pow(this->Dt, 4), pow(this->Dt, 3), pow(this->Dt, 2), this->Dt, 1,
        0, 0, 0, 0, 1, 0,
        5*pow(this->Dt, 4), 4*pow(this->Dt, 3), 3*pow(this->Dt, 2), 2*this->Dt, 1, 0,
        0, 0, 0, 2, 0, 0,
        20*pow(this->Dt, 3), 12*pow(this->Dt, 2), 6*this->Dt, 2, 0, 0;
  coef = (A.inverse())*L;

  this->a = {coef(0), coef(1), coef(2), coef(3), coef(4), coef(5)};
};

void          Polynomial::update(const double &piIn, const double &pfIn, const double & DtIn, const double & tiIn){
  //TODO update polynomial coefficients
};

const double  Polynomial::p     (const double &t){
  //TODO compute position
  const double td = t - this->ti;

  const double pos = a[0]*pow(td, 5) + a[1]*pow(td, 4) + a[2]*pow(td, 3) + a[3]*pow(td, 2) + a[4]*td + a[5];

  return pos;
};

const double  Polynomial::dp    (const double &t){
  //TODO compute velocity
  const double td = t - this->ti;

  const double vel = a[0]*5*pow(td, 4) + a[1]*4*pow(td, 3) + a[2]*3*pow(td, 2) + a[3]*2*td + a[4];

  return vel;
};

Point2Point::Point2Point(){}

Point2Point::Point2Point(const Eigen::Vector<double, 3> & xi, const Eigen::Quaterniond quati, const Eigen::Vector<double, 3> & xf, const Eigen::Quaterniond quatf, const double & DtIn, const double & tiIn){
  //TODO initialize object and polynomials
  Polynomial polX(xi(0), xf(0), DtIn, tiIn);
  Polynomial polY(xi(1), xf(1), DtIn, tiIn);
  Polynomial polZ(xi(2), xf(2), DtIn, tiIn);

  this->qi = quati;
  this->qf = quatf;
  this->polx = polX;
  this->poly = polY;
  this->polz = polZ;
  this->Dt = DtIn;
  this->ti = tiIn;
}

Eigen::Vector<double, 3> Point2Point::X (const double & time) {
  //TODO compute cartesian position
  return Eigen::Vector<double, 3>(this->polx.p(time), this->poly.p(time), this->polz.p(time));
}

Eigen::Vector<double, 3> Point2Point::dX(const double & time){
  //TODO compute cartesian velocity
  return Eigen::Vector<double, 3>(this->polx.dp(time), this->poly.dp(time), this->polz.dp(time));
}

Eigen::Quaterniond Point2Point::Q(const double & t) {
  const double td = t/this->Dt;
  return this->qi.slerp(td, this->qf);
}

// Eigen::Vector<double, 3> Point2Point::dQ(const double & t) {
// }
