#ifndef ANGULAR_MOMENTUM_KF_HPP
#define ANGULAR_MOMENTUM_KF_HPP

#include <Eigen/Dense>

using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
class AngularMomentumKF
{
public:
   // default constructor
   AngularMomentumKF() {}
   // constructor with parameters
   AngularMomentumKF(double pos_var, double Lmeas_var)
   {
      this->pos_var = pos_var;
      this->Lmeas_var = Lmeas_var;
   }

   Vector2d Update(double dt, Vector2d Lmeas, Vector2d uk);
   void Reset() { initialized = false; }

private:
   double pos_var = 1;
   double Lmeas_var = 0.1;
   double grav = 9.81;
   Matrix2d Pk;

   Matrix2d Ak = Matrix2d::Identity();
   MatrixXd Bk = Matrix2d::Identity();
   Matrix2d Ck = Matrix2d::Identity();

   Matrix2d Rk = Matrix2d::Identity();
   Matrix2d Qk = Matrix2d::Identity();

   Vector2d L_old;

   bool initialized = false;
};

#endif // ANGULAR_MOMENTUM_KF_HPP
