#ifndef ANGULAR_MOMENTUM_KF_HPP
#define ANGULAR_MOMENTUM_KF_HPP

#include <Eigen/Dense>

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

   Eigen::Vector2d Update(double dt, Eigen::Vector2d Lmeas, Eigen::Vector2d uk);
   void Reset() { initialized = false; }

private:
   double pos_var = 1;
   double Lmeas_var = 0.1;
   double grav = 9.81;
   Eigen::Matrix2d Pk;

   Eigen::Matrix2d Ak = Eigen::Matrix2d::Identity();
   Eigen::MatrixXd Bk = Eigen::Matrix2d::Identity();
   Eigen::Matrix2d Ck = Eigen::Matrix2d::Identity();

   Eigen::Matrix2d Rk = Eigen::Matrix2d::Identity();
   Eigen::Matrix2d Qk = Eigen::Matrix2d::Identity();

   Eigen::Vector2d L_old;

   bool initialized = false;
};

#endif // ANGULAR_MOMENTUM_KF_HPP
