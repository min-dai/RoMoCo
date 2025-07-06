#pragma once



#include <Eigen/Dense>


struct h1_sensor
{
    /* data */
    Eigen::VectorXd encoders_pos_pinocchio_order;
    Eigen::VectorXd encoders_vel_pinocchio_order;


    //directly from mujoco sim
    Eigen::Vector3d base_lin_pos;
    Eigen::Vector3d base_lin_vel;
    Eigen::Vector4d base_ang_quat;
    Eigen::Vector3d base_ang_vel;

};


void getAllJointStateFromSensorMujoco(const h1_sensor &sensor, Eigen::VectorXd &q, Eigen::VectorXd &qdot);

