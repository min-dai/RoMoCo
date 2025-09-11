#ifndef ROMOCO_PLANNER_MLIP_FLAT_HPP
#define ROMOCO_PLANNER_MLIP_FLAT_HPP

#include <romoco_core/biped_constants.hpp>
#include <Eigen/Dense>
#include "romoco_planner/lip_base.hpp"

namespace romoco
{
   class MLIPFlat : public LIPBase
   {
   public:
      MLIPFlat(bool useMomentum_ = true);
      void set_use_dcm(bool useDCM)
      {
         use_dcm_ = useDCM;
      }
      void Init(double z0, double Ts, double Td, int orbitPeriod, double vel, double stepwidth);

      int orbitPeriod = 0; // 1 is P1, 2 is P2

      struct Params
      {
         double l = 0;

         Eigen::Matrix3d A;

      } params;
      Eigen::Vector2d Kdeadbeat;
      Eigen::Vector2d Klqr;

   private:
      bool use_dcm_ = false;
      double Ts_, Td_, T_;
   }
} // namespace romoco

#endif // ROMOCO_PLANNER_MLIP_FLAT_HPP