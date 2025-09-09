#include "romoco_utils/angular_momentum_kf.hpp"

Eigen::Vector2d AngularMomentumKF::Update(double dt, Eigen::Vector2d Lmeas, Eigen::Vector2d uk)
{
   if (!initialized)
   {
      Qk = Lmeas_var * Eigen::Matrix2d::Identity();
      Pk = Qk;
      initialized = true;
      L_old = Lmeas;
   }

   Rk = pow(dt * grav * pos_var, 2) * Eigen::Matrix2d::Identity();

   Eigen::Vector2d L_pri, L_post;
   L_pri = Ak * L_old + Bk * uk;

   Pk = Ak * Pk * Ak.transpose() + Rk;
   Eigen::Matrix2d Sk = Ck * Pk * Ck.transpose() + Qk;
   Eigen::Matrix2d K = Pk * Ck.transpose() * Sk.inverse();
   L_post = L_pri + K * (Lmeas - Ck * L_pri);
   Pk = (Eigen::Matrix2d::Identity() - K * Ck) * Pk;

   L_old = L_post;

   return L_post;
}