#ifndef ROMOCO_LIP_BASE_HPP
#define ROMOCO_LIP_BASE_HPP

#include <Eigen/Dense>
#include "romoco_core/biped_constants.hpp"
#include <memory>

namespace romoco
{
   /**
    * @class LIPBase
    * @brief A class for solving the Linear Inverted Pendulum (LIP) model.
    * @ingroup group_controllers
    */
   class LIPBase
   {
   public:
      LIPBase(bool use_momentum) : use_momentum_(use_momentum) {};
      virtual ~LIPBase() = default; // Virtual destructor for inheritance

      double lambda() { return sqrt(grav / z0_); }
      double z0() const { return z0_; }

      Eigen::Vector2d SolveLIP(double t, Eigen::Vector2d X0);

   protected:
      bool use_momentum_ = true;
      double z0_ = 0.0; // CoM height

      Eigen::Matrix2d A2_ss();
      Eigen::Matrix2d A2_ds_constant_vel();
      Eigen::Matrix3d A3();

      double SolveOrbitalEnergy(double p, double Ly, double z0, double grav);
      Eigen::Vector2d SolveDeadbeatGain(Eigen::Matrix2d A, Eigen::Vector2d B);
   };

} // namespace romoco
#endif // ROMOCO_LIP_BASE_HPP