#include "biped_core/mujoco_interface_base.hpp"
#include "biped_core/prep_proprioception.hpp"

#include <iostream>
void MujocoInterfaceBase::GetAllJointStateFromSensorMujoco(Eigen::VectorXd &q, Eigen::VectorXd &qdot)
{
   q = full_proprioception_.q;
   qdot = full_proprioception_.qdot;
}

void MujocoInterfaceBase::ReadAndEstimate()
{
   full_proprioception_ = GetBipedProprioceptionFromSensorDataPostEstimation(sensor_);
   loco_proprioception_ = full_proprioception_.GetIndex(loco_proprio_indices_);
   pd_proprioception_ = full_proprioception_.GetIndex(pd_proprio_indices_);
   std::cout << "READ" << std::endl;
}

void MujocoInterfaceBase::SendPacket()
{
   torque_loco_ = loco_motor_commands_.SolveFullTorque(loco_proprioception_.q, loco_proprioception_.qdot);
   torque_pd_ = pd_motor_commands_.SolveFullTorque(pd_proprioception_.q, pd_proprioception_.qdot);

   Step(torque_loco_, torque_pd_);
   std::cout << "SENT + STEP" << std::endl;
}