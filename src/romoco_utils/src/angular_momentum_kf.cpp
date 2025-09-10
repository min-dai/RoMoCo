#include "romoco_utils/angular_momentum_kf.hpp"

Eigen::Vector2d AngularMomentumKF::Update(double dt, Eigen::Vector2d Lmeas, Eigen::Vector2d uk)
{
   if (!initialized_)
   {
      Qk_ = Lmeas_var_ * Eigen::Matrix2d::Identity();
      Pk_ = Qk_;
      initialized_ = true;
      L_old_ = Lmeas;
   }

   Rk_ = pow(dt * grav_ * pos_var_, 2) * Eigen::Matrix2d::Identity();

   Eigen::Vector2d L_pri, L_post;
   L_pri = Ak_ * L_old_ + Bk_ * uk;

   Pk_ = Ak_ * Pk_ * Ak_.transpose() + Rk_;
   Eigen::Matrix2d Sk = Ck_ * Pk_ * Ck_.transpose() + Qk_;
   Eigen::Matrix2d K = Pk_ * Ck_.transpose() * Sk.inverse();
   L_post = L_pri + K * (Lmeas - Ck_ * L_pri);
   Pk_ = (Eigen::Matrix2d::Identity() - K * Ck_) * Pk_;

   L_old_ = L_post;

   return L_post;
}