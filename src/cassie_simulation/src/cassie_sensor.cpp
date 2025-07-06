#include "cassie_sensor.hpp"
#include <iostream>

// todo: use imu
// use mujoco qpos and qvel for base pos, quat, and vel
// no imu used(as imu frame may not coincide with the base frame)
void getAllJointStateFromSensorMujoco(const cassie_sensor &sensor, Eigen::VectorXd &q, Eigen::VectorXd &qdot)
{
    Eigen::Quaterniond quat;
    quat.w() = sensor.cassie_out.pelvis.vectorNav.orientation[0];
    quat.x() = sensor.cassie_out.pelvis.vectorNav.orientation[1];
    quat.y() = sensor.cassie_out.pelvis.vectorNav.orientation[2];
    quat.z() = sensor.cassie_out.pelvis.vectorNav.orientation[3];

    Eigen::EulerAnglesZYXd euler = eulerZYX(quat);

    Eigen::VectorXd qfloating6zyx(6);
    qfloating6zyx.segment(BasePosX, 3) = sensor.base_lin_pos;
    qfloating6zyx(BaseRotZ) = euler.alpha(); // yaw
    qfloating6zyx(BaseRotY) = euler.beta();  // pitch
    qfloating6zyx(BaseRotX) = euler.gamma(); // roll

    Eigen::VectorXd dqfloating6zyx(6);

    dqfloating6zyx << sensor.base_lin_vel, angularVel2EulerRate(euler, sensor.base_ang_vel);

    Eigen::VectorXd qleg(12);
    qleg << sensor.cassie_out.leftLeg.hipRollDrive.position,
        sensor.cassie_out.leftLeg.hipYawDrive.position,
        sensor.cassie_out.leftLeg.hipPitchDrive.position,
        sensor.cassie_out.leftLeg.kneeDrive.position,
        sensor.cassie_out.leftLeg.tarsusJoint.position,
        sensor.cassie_out.leftLeg.footJoint.position,
        sensor.cassie_out.rightLeg.hipRollDrive.position,
        sensor.cassie_out.rightLeg.hipYawDrive.position,
        sensor.cassie_out.rightLeg.hipPitchDrive.position,
        sensor.cassie_out.rightLeg.kneeDrive.position,
        sensor.cassie_out.rightLeg.tarsusJoint.position,
        sensor.cassie_out.rightLeg.footJoint.position;

    Eigen::VectorXd dqleg(12);
    dqleg << sensor.cassie_out.leftLeg.hipRollDrive.velocity,
        sensor.cassie_out.leftLeg.hipYawDrive.velocity,
        sensor.cassie_out.leftLeg.hipPitchDrive.velocity,
        sensor.cassie_out.leftLeg.kneeDrive.velocity,
        sensor.cassie_out.leftLeg.tarsusJoint.velocity,
        sensor.cassie_out.leftLeg.footJoint.velocity,
        sensor.cassie_out.rightLeg.hipRollDrive.velocity,
        sensor.cassie_out.rightLeg.hipYawDrive.velocity,
        sensor.cassie_out.rightLeg.hipPitchDrive.velocity,
        sensor.cassie_out.rightLeg.kneeDrive.velocity,
        sensor.cassie_out.rightLeg.tarsusJoint.velocity,
        sensor.cassie_out.rightLeg.footJoint.velocity;

    q << qfloating6zyx, qleg;
    qdot << dqfloating6zyx, dqleg;

    // std::cout << "q: " << q.transpose() << std::endl;
    // std::cout << "qdot: " << qdot.transpose() << std::endl;
}

void getAllJointStateFromSensorMujoco(const Eigen::Vector3d base_lin_pos, const Eigen::Vector3d base_lin_vel, const cassie_out_t &cassie_out, Eigen::VectorXd &q, Eigen::VectorXd &qdot)
{
    Eigen::Quaterniond quat;
    quat.w() = cassie_out.pelvis.vectorNav.orientation[0];
    quat.x() = cassie_out.pelvis.vectorNav.orientation[1];
    quat.y() = cassie_out.pelvis.vectorNav.orientation[2];
    quat.z() = cassie_out.pelvis.vectorNav.orientation[3];
    Eigen::EulerAnglesZYXd euler = eulerZYX(quat);

    Eigen::Vector3d angularrate;
    angularrate << cassie_out.pelvis.vectorNav.angularVelocity[0], cassie_out.pelvis.vectorNav.angularVelocity[1], cassie_out.pelvis.vectorNav.angularVelocity[2];

    Eigen::VectorXd qfloating6zyx(6);
    qfloating6zyx.segment(BasePosX, 3) = base_lin_pos;
    qfloating6zyx(BaseRotZ) = euler.alpha(); // yaw
    qfloating6zyx(BaseRotY) = euler.beta();  // pitch
    qfloating6zyx(BaseRotX) = euler.gamma(); // roll

    Eigen::VectorXd dqfloating6zyx(6);
    dqfloating6zyx << base_lin_vel, angularVel2EulerRate(euler, angularrate);

    Eigen::VectorXd qleg(12);
    qleg << cassie_out.leftLeg.hipRollDrive.position,
        cassie_out.leftLeg.hipYawDrive.position,
        cassie_out.leftLeg.hipPitchDrive.position,
        cassie_out.leftLeg.kneeDrive.position,
        cassie_out.leftLeg.tarsusJoint.position,
        cassie_out.leftLeg.footJoint.position,
        cassie_out.rightLeg.hipRollDrive.position,
        cassie_out.rightLeg.hipYawDrive.position,
        cassie_out.rightLeg.hipPitchDrive.position,
        cassie_out.rightLeg.kneeDrive.position,
        cassie_out.rightLeg.tarsusJoint.position,
        cassie_out.rightLeg.footJoint.position;

    Eigen::VectorXd dqleg(12);
    dqleg << cassie_out.leftLeg.hipRollDrive.velocity,
        cassie_out.leftLeg.hipYawDrive.velocity,
        cassie_out.leftLeg.hipPitchDrive.velocity,
        cassie_out.leftLeg.kneeDrive.velocity,
        cassie_out.leftLeg.tarsusJoint.velocity,
        cassie_out.leftLeg.footJoint.velocity,
        cassie_out.rightLeg.hipRollDrive.velocity,
        cassie_out.rightLeg.hipYawDrive.velocity,
        cassie_out.rightLeg.hipPitchDrive.velocity,
        cassie_out.rightLeg.kneeDrive.velocity,
        cassie_out.rightLeg.tarsusJoint.velocity,
        cassie_out.rightLeg.footJoint.velocity;

        q << qfloating6zyx, qleg;
        qdot << dqfloating6zyx, dqleg;
}
