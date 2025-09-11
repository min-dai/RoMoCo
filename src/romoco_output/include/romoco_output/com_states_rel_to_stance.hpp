#ifndef BIPED_CORE_COM_STATES_REL_TO_STANCE_HPP
#define BIPED_CORE_COM_STATES_REL_TO_STANCE_HPP

#include "romoco_utils/angular_momentum_kf.hpp"

#include <Eigen/Dense>

namespace romoco
{
   /**
    * @class ComStatesRelToStance
    * @brief Computes the center of mass (COM) states relative to the stance foot frame for a biped robot.
    * @ingroup group_controller
    * This class calculates the position, velocity, and angular momentum of the robot's center of mass (COM)
    * relative to the stance foot frame. It utilizes a Kalman filter, AngularMomentumKF, to estimate the angular momentum.
    * The computed states include the COM position, COM velocity, angular momentum about the COM, and angular momentum about the pivot point.
    * The class provides methods to reset the Kalman filter and compute the states based on the robot's current configuration and desired yaw.
    */
class ComStatesRelToStance
{
public:
   struct States
   {
      Eigen::Vector3d pCOM;
      Eigen::Vector3d vCOM;
      Eigen::Vector3d Lcom;
      Eigen::Vector3d Lpivot;
   } states;

   void Reset()
   {
      AMkf.Reset();
   }

   void compute(
       const Eigen::Vector3d &com_pos_world,
       const Eigen::Vector3d &com_vel_world,
       const Eigen::Vector3d &stance_pos_world,
       const Eigen::Vector3d &normalized_centroidal_ang_momentum_world,
       const double &yaw_target,
       const double &dt);

private:
   AngularMomentumKF AMkf;
};

} // namespace romoco

#endif // BIPED_CORE_COM_STATES_REL_TO_STANCE_HPP
