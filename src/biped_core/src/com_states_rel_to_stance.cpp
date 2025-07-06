#include "biped_core/com_states_rel_to_stance.hpp"
#include "biped_core/biped_constants.hpp"
void ComStatesRelToStance::compute(
    const Eigen::Vector3d& com_pos_world,
    const Eigen::Vector3d& com_vel_world,
    const Eigen::Vector3d& stance_pos_world,
    const Eigen::Vector3d& normalized_centroidal_ang_momentum_world,
    const double& yaw_target,
    const double& dt) {
   // Rotate the COM position and velocity from world frame to the target yaw frame
   Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(-yaw_target, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
   states.pCOM = rotation_matrix * (com_pos_world - stance_pos_world);
   states.vCOM = rotation_matrix * com_vel_world;

   // Compute the angular momentum in the target yaw frame
   states.Lcom = rotation_matrix * normalized_centroidal_ang_momentum_world;
   
   // Compute the pivot point in the target yaw frame
   Eigen::Vector3d angular_momentum_cross = states.pCOM.cross(states.vCOM);
   Eigen::Vector3d Lmeas;
   Lmeas << -states.Lcom.x() - angular_momentum_cross.x(), states.Lcom.y() + angular_momentum_cross.y(), states.Lcom.z() + angular_momentum_cross.z();

   Vector2d uk;
   uk << dt * grav * states.pCOM.y(),
       dt * grav * states.pCOM.x();

   Vector2d Lpivot2 = AMkf.Update(dt, Lmeas.head(2), uk);

   states.Lpivot << Lpivot2.x(), Lpivot2.y(), Lmeas.z();
}