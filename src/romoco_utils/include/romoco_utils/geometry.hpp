#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>


/**
 * @file geometry.hpp
 * @brief Utility functions for geometric computations in the romoco namespace.
 *
 * Provides functions for converting between quaternions, rotation matrices, and Euler angles,
 * as well as utilities for skew-symmetric matrices and angular velocity transformations.
 */

namespace romoco{

/**
 * @brief Converts a quaternion to ZYX Euler angles.
 * @param q Quaternion representing rotation.
 * @return Euler angles in ZYX convention.
 */
Eigen::EulerAnglesZYXd eulerZYX(const Eigen::Quaterniond &q);

/**
 * @brief Converts a rotation matrix to ZYX Euler angles.
 * @param R Rotation matrix.
 * @return Euler angles in ZYX convention.
 */
Eigen::EulerAnglesZYXd eulerZYX(const Eigen::Matrix3d &R);

/**
 * @brief Converts a quaternion to XYZ Euler angles.
 * @param q Quaternion representing rotation.
 * @return Euler angles in XYZ convention.
 */
Eigen::EulerAnglesXYZd eulerXYZ(const Eigen::Quaterniond &q);

/**
 * @brief Converts a rotation matrix to XYZ Euler angles.
 * @param R Rotation matrix.
 * @return Euler angles in XYZ convention.
 */
Eigen::EulerAnglesXYZd eulerXYZ(const Eigen::Matrix3d &R);

/**
 * @brief Computes the skew-symmetric matrix of a 3D vector.
 * @param v 3D vector.
 * @return Skew-symmetric matrix corresponding to the vector.
 */
Eigen::Matrix3d skew(const Eigen::Vector3d &v);

/**
 * @brief Converts angular velocity to Euler angle rates for ZYX Euler angles.
 * @param euler Current ZYX Euler angles.
 * @param w Angular velocity vector.
 * @return Euler angle rates corresponding to the angular velocity.
 */
Eigen::Vector3d angularVel2EulerRate(const Eigen::EulerAnglesZYXd &euler, const Eigen::Vector3d &w);

} // namespace romoco

#endif // GEOMETRY_HPP
