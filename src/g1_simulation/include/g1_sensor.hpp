#ifndef G1_SIMULATION_INCLUDE_G1_SENSOR_HPP
#define G1_SIMULATION_INCLUDE_G1_SENSOR_HPP


#include <Eigen/Dense>


struct g1_sensor
{
    /* data */
    Eigen::Vector3d imu_gyro;
    Eigen::Vector3d imu_accelerometer;
    Eigen::VectorXd imu_framequat;

    Eigen::VectorXd encoders_pos_pinocchio_order;
    Eigen::VectorXd encoders_vel_pinocchio_order;



    //directly from mujoco sim
    Eigen::Vector3d base_lin_pos;
    Eigen::Vector3d base_lin_vel;
    Eigen::Vector4d base_ang_quat;
    Eigen::Vector3d base_ang_vel;

};


void getAllJointStateFromSensorMujoco(const g1_sensor &sensor, Eigen::VectorXd &q, Eigen::VectorXd &qdot);

#endif // G1_SIMULATION_INCLUDE_G1_SENSOR_HPP