#ifndef BIPED_CORE_COM_STATES_REL_TO_STANCE_HPP
#define BIPED_CORE_COM_STATES_REL_TO_STANCE_HPP

#include "biped_utils/angular_momentum_kf.hpp"

#include <Eigen/Dense>
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

#endif // BIPED_CORE_COM_STATES_REL_TO_STANCE_HPP
