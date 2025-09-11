   #include "romoco_planner/lip_base.hpp"
#include <unsupported/Eigen/MatrixFunctions> //matrix exponential
   namespace romoco
   {
      Eigen::Matrix2d LIPBase::A2_ss()
      {
         Eigen::Matrix2d A2;
         if (use_momentum_)
         {
            A2 << 0, 1 / z0_,
                grav, 0;
         }
         else
         {
            A2 << 0, 1,
                grav / z0_, 0;
         }
         return A2;
      }
      Eigen::Matrix2d LIPBase::A2_ds_constant_vel()
      {
         Eigen::Matrix2d A2;
         if (use_momentum_)
         {
            A2 << 0, 1 / z0_,
                0, 0;
         }
         else
         {
            A2 << 0, 1,
                0, 0;
         }
         return A2;
      }
      Eigen::Matrix3d LIPBase::A3()
      {
         Eigen::Matrix3d A3;
         if (use_momentum_)
         {
            A3 << 0, 1 / z0_, 0,
            grav, 0, -grav,
            0, 0, 0;
         }
         else
         {
            A3 << 0, 1, 0,
                grav / z0_, 0, -grav / z0_,
                0, 0, 0;
         }
         return A3;
      }

      Eigen::Vector2d LIPBase::SolveLIP(double t, Eigen::Vector2d X0)
      {
         // given X(0), solve for X(t)
         Eigen::Vector2d sol;
         Eigen::Matrix2d A2 = A2_ss();
         sol = (t * A2).exp() * X0;
         return sol;
      }



      Eigen::Vector2d LIPBase::SolveDeadbeatGain(Eigen::Matrix2d A, Eigen::Vector2d B)
      {
         // explict solution for 2-by-2 matrix only
         Eigen::Matrix2d Atmp;
         Atmp << -B(0), -B(1),
             A(1, 1) * B(0) - A(0, 1) * B(1), A(0, 0) * B(1) - A(1, 0) * B(0);
         Eigen::MatrixXd Btmp(2, 1);
         Btmp << A(0, 0) + A(1, 1),
             A(0, 1) * A(1, 0) - A(0, 0) * A(1, 1);

         Eigen::MatrixXd Ktmp = Atmp.inverse() * Btmp;
         Eigen::Vector2d Kdeadbeat(Ktmp(0, 0), Ktmp(1, 0));
         return Kdeadbeat;
      }
   }