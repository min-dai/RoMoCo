#ifndef ANGULAR_MOMENTUM_KF_HPP
#define ANGULAR_MOMENTUM_KF_HPP

#include <Eigen/Dense>

/**
 * @class AngularMomentumKF
 * @ingroup group_utils
 * @brief Implements a simple Kalman Filter for angular momentum estimation.
 *
 * This class provides methods to estimate angular momentum using a Kalman Filter approach.
 * It supports initialization with custom process and measurement noise variances, and
 * allows updating the filter state with new measurements and control inputs.
 */
class AngularMomentumKF
{
public:
   // default constructor
   AngularMomentumKF() {}
   // constructor with parameters
   AngularMomentumKF(double pos_var, double Lmeas_var)
   {
      this->pos_var_ = pos_var;
      this->Lmeas_var_ = Lmeas_var;
   }

   Eigen::Vector2d Update(double dt, Eigen::Vector2d Lmeas, Eigen::Vector2d uk);
   void Reset() { initialized_ = false; }

private:
   double pos_var_ = 1;
   double Lmeas_var_ = 0.1;
   double grav_ = 9.81;
   Eigen::Matrix2d Pk_;

   Eigen::Matrix2d Ak_ = Eigen::Matrix2d::Identity();
   Eigen::MatrixXd Bk_ = Eigen::Matrix2d::Identity();
   Eigen::Matrix2d Ck_ = Eigen::Matrix2d::Identity();

   Eigen::Matrix2d Rk_ = Eigen::Matrix2d::Identity();
   Eigen::Matrix2d Qk_ = Eigen::Matrix2d::Identity();

   Eigen::Vector2d L_old_;

   bool initialized_ = false;
};

#endif // ANGULAR_MOMENTUM_KF_HPP
