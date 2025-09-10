#ifndef BIPED_PROPRIOCEPTION_HPP
#define BIPED_PROPRIOCEPTION_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace romoco
{
struct RawSensorData
{
   //initialize lin_pos as NAN
   Eigen::Vector3d base_lin_pos = Eigen::Vector3d::Constant(NAN);
   Eigen::Quaterniond base_ang_quat;
   Eigen::Vector3d base_ang_vel;
   Eigen::VectorXd encoders_pos_pinocchio_order;
   Eigen::VectorXd encoders_vel_pinocchio_order;

   void ResizeAll(int dof)
   {
       encoders_pos_pinocchio_order.resize(dof);
       encoders_vel_pinocchio_order.resize(dof);
   }

   //overload <<
   friend std::ostream &operator<<(std::ostream &os, const RawSensorData &data)
   {
       os << "Base Linear Position: " << data.base_lin_pos.transpose() << "\n";
       os << "Base Angular Quaternion: " << data.base_ang_quat.w() << " " << data.base_ang_quat.x() << " " << data.base_ang_quat.y() << " " << data.base_ang_quat.z() << "\n";
       os << "Base Angular Velocity: " << data.base_ang_vel.transpose() << "\n";
       os << "Encoders Position (Pinocchio Order): " << data.encoders_pos_pinocchio_order.transpose() << "\n";
       os << "Encoders Velocity (Pinocchio Order): " << data.encoders_vel_pinocchio_order.transpose() << "\n";
       return os;
   }
};

struct BipedEstimation
{
   Eigen::Vector3d base_lin_vel;
   BipedEstimation() : base_lin_vel(Eigen::Vector3d::Zero()) {}
};

struct RawSensorDataHardware : RawSensorData
{
   Eigen::Vector3d base_lin_acc;

   friend std::ostream &operator<<(std::ostream &os, const RawSensorDataHardware &data)
   {
       os << static_cast<const RawSensorData &>(data);
       os << "Base Linear Acceleration: " << data.base_lin_acc.transpose() << "\n";
       return os;
   }
};

struct SensorDataPostEstimation : RawSensorData 
{
   Eigen::Vector3d base_lin_vel;
   //overload << operator
   friend std::ostream &operator<<(std::ostream &os, const SensorDataPostEstimation &data)
   {
       os << static_cast<const RawSensorData &>(data);
       os << "Base Linear Velocity: " << data.base_lin_vel.transpose() << "\n";
       return os;
   }
};

struct BipedProprioception
{
   Eigen::VectorXd q;    // floating base + encoders, in pinocchio order
   Eigen::VectorXd qdot; // floating base + encoders velocities, in pinocchio order
   friend std::ostream &operator<<(std::ostream &os, const BipedProprioception &data)
   {
       os << "Proprioception q: " << data.q.transpose() << "\n";
       os << "Proprioception qdot: " << data.qdot.transpose() << "\n";
       return os;
   }
   BipedProprioception GetIndex(const std::vector<int> &indices) const
   {
       BipedProprioception result;
       result.q = q(indices);
       result.qdot = qdot(indices);
       return result;
   }
   void ResizeAll(int dof)
   {
       q.resize(dof);
       qdot.resize(dof);
   }
   void ZeroAll(int dof)
   {
       q = Eigen::VectorXd::Zero(dof);
       qdot = Eigen::VectorXd::Zero(dof);
   }
};

} // namespace romoco

#endif // BIPED_PROPRIOCEPTION_HPP