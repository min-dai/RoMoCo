#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>
#include <fstream>

Eigen::EulerAnglesZYXd eulerZYX(const Eigen::Quaterniond &q);

Eigen::EulerAnglesZYXd eulerZYX(const Eigen::Matrix3d &R);

Eigen::EulerAnglesXYZd eulerXYZ(const Eigen::Quaterniond &q);

Eigen::EulerAnglesXYZd eulerXYZ(const Eigen::Matrix3d &R);

Eigen::Matrix3d skew(const Eigen::Vector3d &v);

Eigen::Vector3d angularVel2EulerRate(const Eigen::EulerAnglesZYXd &euler, const Eigen::Vector3d &w);

#endif // GEOMETRY_HPP
