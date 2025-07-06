#include "h1_sensor.hpp"
#include <iostream>
#include "biped_core/biped_constants.hpp"
#include "biped_utils/geometry.hpp"


// todo: use imu
//use mujoco qpos and qvel for base pos, quat, and vel
//no imu used(as imu frame may not coincide with the base frame)
void getAllJointStateFromSensorMujoco(const h1_sensor &sensor, Eigen::VectorXd &q, Eigen::VectorXd &qdot)
{
    Eigen::Quaterniond quat;
    quat.w() = sensor.base_ang_quat(0);
    quat.x() = sensor.base_ang_quat(1);
    quat.y() = sensor.base_ang_quat(2);
    quat.z() = sensor.base_ang_quat(3);

    Eigen::EulerAnglesZYXd euler = eulerZYX(quat);


    Eigen::VectorXd qfloating6zyx(6);
    qfloating6zyx.segment(BasePosX, 3) = sensor.base_lin_pos;
    qfloating6zyx(BaseRotZ) = euler.alpha(); //yaw
    qfloating6zyx(BaseRotY) = euler.beta();  // pitch
    qfloating6zyx(BaseRotX) = euler.gamma(); // roll



    Eigen::VectorXd dqfloating6zyx(6); 

    dqfloating6zyx << sensor.base_lin_vel, angularVel2EulerRate(euler, sensor.base_ang_vel);

    // std::cout << "qfloating6zyx: " << qfloating6zyx.transpose() << std::endl;
    // std::cout << "dqfloating6zyx: " << dqfloating6zyx.transpose() << std::endl;

    q<< qfloating6zyx, sensor.encoders_pos_pinocchio_order;
    qdot<< dqfloating6zyx, sensor.encoders_vel_pinocchio_order;

    // std::cout << "q: " << q.transpose() << std::endl;
    // std::cout << "qdot: " << qdot.transpose() << std::endl;

}