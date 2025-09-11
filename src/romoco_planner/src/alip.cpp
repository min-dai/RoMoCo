#include "romoco_planner/alip.hpp"
#include <cmath>

namespace romoco
{

   ALIP::ALIP() : LIPBase(true)
   {
   }

   void ALIP::Init(double z0, double Ts, int orbitPeriod, double vel, double stepwidth)
   {

      orbitPeriod = orbitPeriod;
      UpdateALIP(z0, Ts);
      UpdateDesiredWalking(vel, stepwidth);
   }

   void ALIP::UpdateALIP(double z0, double Ts)
   {
      Ts_ = Ts;
      z0_ = z0;
   }

   void ALIP::UpdateDesiredWalking(double vel, double stepwidth)
   {
      // Initialize P1 and P2 parameters
      Lydes_ = z0_ * vel;
      double lam = lambda();

      Lxdes_right_ = -1 / 2. * z0_ * stepwidth * (lam * sinh(Ts_ * lam)) / (1. + cosh(lam * Ts_));
      Lxdes_left_ = -Lxdes_right_;
   }

   double ALIP::SolveDesiredStepSizeP1(Eigen::Vector2d state)
   {
      // Solve for the desired step size in P1

      double stepLength;
      double lam = lambda();
      double T = Ts_;
      stepLength = (cosh(lam * T) * state(1) + z0_ * lam * sinh(lam * T) * state(0) - Lydes_)/(z0_ * lam * sinh(lam * T) );
      return stepLength;
   }

   double ALIP::SolveDesiredStepSizeP2(Eigen::Vector2d state, StanceStatus stanceLeg)
   {
      // Solve for the desired step size in P2
      double Lxdes = (stanceLeg == StanceStatus::LeftStance) ? Lxdes_left_ : Lxdes_right_;
      double stepLength;
      double lam = lambda();
      double T = Ts_;
      stepLength = (cosh(lam * T) * state(1) + z0_ * lam * sinh(lam * T) * state(0) - Lxdes)/(z0_ * lam * sinh(lam * T) );
      return stepLength;
   }

} // namespace romoco