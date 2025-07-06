#pragma once


#include <Eigen/Dense>
#include "biped_core/biped_constants.hpp"
#include "biped_utils/geometry.hpp"
#include "cassie_interface/cassie_out_t.h"
struct cassie_sensor
{
    // /* data */
    // Eigen::Vector3d imu_gyro;
    // Eigen::Vector3d imu_accelerometer;
    // Eigen::VectorXd imu_framequat;

    cassie_out_t cassie_out;


    //directly from mujoco sim
    Eigen::Vector3d base_lin_pos;
    Eigen::Vector3d base_lin_vel;
    Eigen::Vector4d base_ang_quat;
    Eigen::Vector3d base_ang_vel;

};


void getAllJointStateFromSensorMujoco(const cassie_sensor &sensor, Eigen::VectorXd &q, Eigen::VectorXd &qdot);

void getAllJointStateFromSensorMujoco(const Eigen::Vector3d base_lin_pos, const Eigen::Vector3d base_lin_vel, const cassie_out_t &cassie_out, Eigen::VectorXd &q, Eigen::VectorXd &qdot);