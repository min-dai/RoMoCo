#ifndef BIPED_PROPRIOCEPTION_HPP
#define BIPED_PROPRIOCEPTION_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
struct RawSensorData
{
   Eigen::Vector3d base_lin_pos; //optional
   Eigen::Quaterniond base_ang_quat;
   Eigen::Vector3d base_ang_vel;
   Eigen::VectorXd encoders_pos_pinocchio_order;
   Eigen::VectorXd encoders_vel_pinocchio_order;

   //overload <<
   friend std::ostream &operator<<(std::ostream &os, const RawSensorData &data)
   {
       os << "Base Linear Position: " << data.base_lin_pos.transpose() << "\n";
       os << "Base Angular Quaternion: " << data.base_ang_quat.coeffs().transpose() << "\n";
       os << "Base Angular Velocity: " << data.base_ang_vel.transpose() << "\n";
       os << "Encoders Position (Pinocchio Order): " << data.encoders_pos_pinocchio_order.transpose() << "\n";
       os << "Encoders Velocity (Pinocchio Order): " << data.encoders_vel_pinocchio_order.transpose() << "\n";
       return os;
   }
};

struct BipedEstimation
{
   Eigen::Vector3d base_lin_vel;
};

struct RawSensorDataHardware : RawSensorData
{
   Eigen::Vector3d base_lin_acc;
};

struct SensorDataPostEstimation : RawSensorData
{
   Eigen::Vector3d base_lin_vel;
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
};

#endif // BIPED_PROPRIOCEPTION_HPP