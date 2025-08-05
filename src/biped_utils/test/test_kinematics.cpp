#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "biped_utils/geometry.hpp"
#include "biped_utils/kinematics.hpp"

TEST(Kinematics1DTest, ConstructionAndArithmetic) {
    Kinematics1D k1(2, 2);
    k1.position << 1.0, 2.0;
    k1.velocity << 0.5, 1.5;
    k1.jacobian << 1.0, 0.0,
                   0.0, 1.0;
    k1.dJdq << 0.1, 0.2;

    Kinematics1D k2 = k1;
    auto k3 = k1 + k2;
    EXPECT_EQ(k3.position[0], 2.0);
    EXPECT_EQ(k3.velocity[1], 3.0);

    auto k4 = k3 * 0.5;
    EXPECT_NEAR(k4.position[0], 1.0, 1e-6);
    EXPECT_NEAR(k4.velocity[1], 1.5, 1e-6);
}

TEST(Kinematics3DTest, CrossAndDotProduct) {
    Kinematics3D k1(3, 3);
    k1.position = Eigen::Vector3d(1, 0, 0);
    k1.velocity = Eigen::Vector3d(0, 1, 0);
    k1.jacobian = Eigen::Matrix3d::Identity();
    k1.dJdq = Eigen::Vector3d(0.1, 0.2, 0.3);

    Kinematics3D k2(3, 3);
    k2.position = Eigen::Vector3d(0, 1, 0);
    k2.velocity = Eigen::Vector3d(0, 0, 1);
    k2.jacobian = 2 * Eigen::Matrix3d::Identity();
    k2.dJdq = Eigen::Vector3d(0.4, 0.5, 0.6);

    auto cross_result = k1.cross(k2);
    EXPECT_NEAR(cross_result.position.z(), 1.0, 1e-6);

    auto dot_result = k1.dot(k2);
    EXPECT_NEAR(dot_result.position[0], 0.0, 1e-6);
}

TEST(Kinematics3DTest, ComponentAccessors) {
    Kinematics3D k(3, 3);
    k.position = Eigen::Vector3d(1, 2, 3);
    k.velocity = Eigen::Vector3d(4, 5, 6);
    k.jacobian = Eigen::MatrixXd::Ones(3, 3);
    k.dJdq = Eigen::Vector3d(0.1, 0.2, 0.3);

    auto x = k.x();
    EXPECT_EQ(x.position[0], 1);
    EXPECT_EQ(x.velocity[0], 4);
    EXPECT_EQ(x.dJdq[0], 0.1);

    auto z = k.z();
    EXPECT_EQ(z.position[0], 3);
    EXPECT_EQ(z.velocity[0], 6);
    EXPECT_EQ(z.dJdq[0], 0.3);
}

TEST(Kinematics3DTest, Rotation) {
    Kinematics3D k(3, 3);
    k.position = Eigen::Vector3d(1, 0, 0);
    k.velocity = Eigen::Vector3d(0, 1, 0);
    k.jacobian = Eigen::Matrix3d::Identity();
    k.dJdq = Eigen::Vector3d(0.1, 0.2, 0.3);

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()); // 90 deg around Z

    auto rotated = k.Rot(R);
    EXPECT_NEAR(rotated.position.y(), 1.0, 1e-6);
    EXPECT_NEAR(rotated.velocity.x(), -1.0, 1e-6);
}
