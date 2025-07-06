#pragma once

#include <Eigen/Dense>
#include <iostream>
#include "biped_utils/geometry.hpp"

struct Kinematics1D
{
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
    Eigen::MatrixXd jacobian;
    Eigen::VectorXd dJdq;

    Kinematics1D() = default;

    Kinematics1D(int rows, int cols)
    {
        position.resize(rows);
        velocity.resize(rows);
        jacobian.resize(rows, cols);
        dJdq.resize(rows);
    }

    Kinematics1D operator+(const Kinematics1D &other) const
    {
        Kinematics1D result;
        result.position = position + other.position;
        result.velocity = velocity + other.velocity;
        result.jacobian = jacobian + other.jacobian;
        result.dJdq = dJdq + other.dJdq;
        return result;
    }

    Kinematics1D operator-(const Kinematics1D &other) const
    {
        Kinematics1D result;
        result.position = position - other.position;
        result.velocity = velocity - other.velocity;
        result.jacobian = jacobian - other.jacobian;
        result.dJdq = dJdq - other.dJdq;
        return result;
    }

    Kinematics1D operator*(double scalar) const
    {
        Kinematics1D result;
        result.position = position * scalar;
        result.velocity = velocity * scalar;
        result.jacobian = jacobian * scalar;
        result.dJdq = dJdq * scalar;
        return result;
    }

    friend Kinematics1D operator*(double scalar, const Kinematics1D &k)
    {
        return k * scalar;
    }

    Kinematics1D operator/(double scalar) const
    {
        if (scalar == 0.0)
        {
            throw std::runtime_error("Division by zero error in Kinematics3D.");
        }
        Kinematics1D result;
        result.position = position / scalar;
        result.velocity = velocity / scalar;
        result.jacobian = jacobian / scalar;
        result.dJdq = dJdq / scalar;
        return result;
    }

    void print() const;
};

struct Kinematics3D
{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::MatrixXd jacobian;
    Eigen::Vector3d dJdq;

    Kinematics3D() = default;

    Kinematics3D(int rows, int cols)
    {
        position.resize(rows);
        velocity.resize(rows);
        jacobian.resize(rows, cols);
        dJdq.resize(rows);
    }

    Kinematics3D operator+(const Kinematics3D &other) const
    {
        Kinematics3D result;
        result.position = position + other.position;
        result.velocity = velocity + other.velocity;
        result.jacobian = jacobian + other.jacobian;
        result.dJdq = dJdq + other.dJdq;
        return result;
    }

    Kinematics3D operator-(const Kinematics3D &other) const
    {
        Kinematics3D result;
        result.position = position - other.position;
        result.velocity = velocity - other.velocity;
        result.jacobian = jacobian - other.jacobian;
        result.dJdq = dJdq - other.dJdq;
        return result;
    }

    Kinematics3D operator*(double scalar) const
    {
        Kinematics3D result;
        result.position = position * scalar;
        result.velocity = velocity * scalar;
        result.jacobian = jacobian * scalar;
        result.dJdq = dJdq * scalar;
        return result;
    }

    friend Kinematics3D operator*(double scalar, const Kinematics3D &k)
    {
        return k * scalar;
    }

    Kinematics3D operator/(double scalar) const
    {
        if (scalar == 0.0)
        {
            throw std::runtime_error("Division by zero error in Kinematics3D.");
        }
        Kinematics3D result;
        result.position = position / scalar;
        result.velocity = velocity / scalar;
        result.jacobian = jacobian / scalar;
        result.dJdq = dJdq / scalar;
        return result;
    }

    Kinematics3D cross(const Kinematics3D &other) const
    {
        Kinematics3D result;
        result.position = position.cross(other.position);
        result.velocity = skew(position) * other.velocity - skew(other.position) * velocity;
        result.jacobian = skew(position) * other.jacobian - skew(other.position) * jacobian;
        result.dJdq = skew(velocity) * other.velocity + skew(position) * other.dJdq - skew(other.velocity) * velocity - skew(other.position) * dJdq;
        return result;
    }

    Kinematics1D dot(const Kinematics3D &other) const
    {
        Kinematics1D result;
        result.position = position.transpose() * other.position;
        result.velocity = position.transpose() * other.velocity + velocity.transpose() * other.position;
        result.jacobian = position.transpose() * other.jacobian + other.position.transpose() * jacobian;
        result.dJdq = 2. * velocity.transpose() * other.velocity + position.transpose() * other.dJdq + other.position.transpose() * dJdq;

        return result;
    }

    Kinematics1D x() const
    {
        Kinematics1D result(1, jacobian.cols());
        result.position << position.x();
        result.velocity << velocity.x();
        result.jacobian << jacobian.row(0);
        result.dJdq << dJdq.x();
        return result;
    }

    Kinematics1D y() const
    {
        Kinematics1D result(1, jacobian.cols());
        result.position << position.y();
        result.velocity << velocity.y();
        result.jacobian << jacobian.row(1);
        result.dJdq << dJdq.y();
        return result;
    }

    Kinematics1D z() const
    {
        Kinematics1D result(1, jacobian.cols());
        result.position << position.z();
        result.velocity << velocity.z();
        result.jacobian << jacobian.row(2);
        result.dJdq << dJdq.z();
        return result;
    }

    Kinematics3D Rot(const Eigen::Matrix3d &R) const
    {
        Kinematics3D result;
        result.position = R * position;
        result.velocity = R * velocity;
        result.jacobian = R * jacobian;
        result.dJdq = R * dJdq;
        return result;
    }

    void print() const;
};
