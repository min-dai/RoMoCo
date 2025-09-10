#include <romoco_core/prep_proprioception.hpp>
#include <romoco_utils/geometry.hpp>

namespace romoco
{

BipedProprioception GetBipedProprioceptionFromSensorDataPostEstimation(const SensorDataPostEstimation &sensor_data)
{
   RawSensorDataHardware raw_sensor_data;
   raw_sensor_data.base_ang_quat = sensor_data.base_ang_quat;
   raw_sensor_data.base_ang_vel = sensor_data.base_ang_vel;
   raw_sensor_data.encoders_pos_pinocchio_order = sensor_data.encoders_pos_pinocchio_order;
   raw_sensor_data.encoders_vel_pinocchio_order = sensor_data.encoders_vel_pinocchio_order;
   BipedEstimation estimation;
   estimation.base_lin_vel = sensor_data.base_lin_vel;
   BipedProprioception proprioception = GetBipedProprioceptionFromRawSensorDataHardware(raw_sensor_data, estimation);
   return proprioception;
}

BipedProprioception GetBipedProprioceptionFromRawSensorDataHardware(const RawSensorDataHardware &raw_sensor_data, const BipedEstimation &estimation)
{
   BipedProprioception proprioception;

   Eigen::EulerAnglesZYXd euler = eulerZYX(raw_sensor_data.base_ang_quat);

   proprioception.q.resize(6 + raw_sensor_data.encoders_pos_pinocchio_order.size());
   proprioception.qdot.resize(6 + raw_sensor_data.encoders_vel_pinocchio_order.size());

   Eigen::VectorXd lin_pos = ((raw_sensor_data.base_lin_pos.array().isNaN()).any()) ? Eigen::Vector3d::Zero() : raw_sensor_data.base_lin_pos;

   proprioception.q << lin_pos, euler.alpha(), euler.beta(), euler.gamma(), raw_sensor_data.encoders_pos_pinocchio_order;
   proprioception.qdot << estimation.base_lin_vel, angularVel2EulerRate(euler, raw_sensor_data.base_ang_vel), raw_sensor_data.encoders_vel_pinocchio_order;


   return proprioception;
}

BipedProprioception GetBipedProprioceptionFromRawSensorDataHardware(const RawSensorDataHardware &raw_sensor_data)
{
   //set an zero estimation
   BipedEstimation estimation;
   return GetBipedProprioceptionFromRawSensorDataHardware(raw_sensor_data, estimation);
}

} // namespace romoco