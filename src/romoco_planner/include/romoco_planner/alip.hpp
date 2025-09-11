#ifndef ALIP_HPP
#define ALIP_HPP

#include <romoco_core/biped_constants.hpp>
#include <Eigen/Dense>
#include "romoco_planner/lip_base.hpp"

namespace romoco
{
   /**
    * @class ALIP
    * @brief A class for solving the Augmented Linear Inverted Pendulum (ALIP) model.
    * @ingroup group_controllers
    */
   class ALIP : public LIPBase
   {

   public:
      ALIP();

      void Init(double z0, double Ts, int orbitPeriod, double vel, double stepwidth);

      int orbitPeriod = 0; // 1 is P1, 2 is P2

      void UpdateALIP(double z0, double Ts);

      void UpdateDesiredWalking(double vel, double stepwidth);

      double SolveDesiredStepSizeP1(Eigen::Vector2d state);

      double SolveDesiredStepSizeP2(Eigen::Vector2d state, StanceStatus stanceLeg);

   private:
      double Ts_;

      double Lydes_;
      double Lxdes_right_, Lxdes_left_;
   };
} // namespace romoco
#endif // ALIP_HPP