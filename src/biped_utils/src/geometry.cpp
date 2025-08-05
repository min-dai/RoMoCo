

#include <biped_utils/geometry.hpp>

Eigen::EulerAnglesZYXd eulerZYX(const Eigen::Quaterniond &q)
{
    Eigen::EulerAnglesZYXd euler;
    // from matlab
    double qw, qx, qy, qz;

    qw = q.w();
    qx = q.x();
    qy = q.y();
    qz = q.z();

    double aSinInput = -2 * (qx * qz - qw * qy);
    if (aSinInput > 1.)
        aSinInput = 1.;
    if (aSinInput < -1.)
        aSinInput = -1.;

    euler.alpha() = atan2(2 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);
    euler.beta() = asin(aSinInput);
    euler.gamma() = atan2(2 * (qy * qz + qw * qx), qw * qw - qx * qx - qy * qy + qz * qz);
    return euler;
}

Eigen::EulerAnglesZYXd eulerZYX(const Eigen::Matrix3d &R)
{
    // did not considered singular case where pitch is pi/2 or -pi/2

    Eigen::EulerAnglesZYXd euler;
    int i = 0;
    int j = 1;
    int k = 2;

    double cySq = R(i, i) * R(i, i) + R(j, i) * R(j, i);
    // bool singular = cySq < 1e-15;
    double cy = sqrt(cySq);

    euler.alpha() = atan2(R(j, i), R(i, i));
    euler.beta() = atan2(-R(k, i), cy);
    euler.gamma() = atan2(R(k, j), R(k, k));

    return euler;
}

Eigen::EulerAnglesXYZd eulerXYZ(const Eigen::Quaterniond &q)
{
    Eigen::EulerAnglesXYZd euler;
    double qx, qy, qz, qw;
    qw = q.w();
    qx = q.x();
    qy = q.y();
    qz = q.z();

    double aSinInput = 2 * (qx * qz + qy * qw);
    if (aSinInput > 1)
        aSinInput = 1;
    if ((aSinInput < -1))
        aSinInput = -1;

    euler.alpha() = atan2(-2 * (qy * qz - qx * qw), pow(qw, 2) - pow(qx, 2) - pow(qy, 2) + pow(qz, 2));
    euler.beta() = asin(aSinInput);
    euler.gamma() = atan2(-2 * (qx * qy - qz * qw), pow(qw, 2) + pow(qx, 2) - pow(qy, 2) - pow(qz, 2));
    return euler;
}

Eigen::EulerAnglesXYZd eulerXYZ(const Eigen::Matrix3d &R)
{
    Eigen::EulerAnglesXYZd euler;
    double y1, x1, z1, x, y, z;
    if (abs(abs(R(2, 0)) - 1.0) > 0.00000001)
    {
        y1 = -asin(R(2, 0));
        // y2 = 3.14159265359 - y1;

        x1 = atan2(R(2, 1) / cos(y1), R(2, 2) / cos(y1));
        // x2 = atan2(R(2,1)/cos(y2), R(2,2)/cos(y2));

        z1 = atan2(R(1, 0) / cos(y1), R(0, 0) / cos(y1));
        // z2 = atan2(R(1,0)/cos(y2), R(0,0)/cos(y2));

        euler.alpha() = x1;
        euler.beta() = y1;
        euler.gamma() = z1;
    }
    else
    {
        z = 0.0;
        if (abs(R(2, 0) + 1.0) < 0.00000001)
        {
            y = 3.14159265359 / 2.0;
            x = z + atan2(R(0, 1), R(0, 2));
        }
        else
        {
            y = -3.14159265359 / 2.0;
            x = -z + atan2(-R(0, 1), -R(0, 2));
        }
        euler.alpha() = x;
        euler.beta() = y;
        euler.gamma() = z;
    }
    return euler;
}

Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d sk;

    sk << 0.0, -v(2), v(1),
        v(2), 0.0, -v(0),
        -v(1), v(0), 0.0;
    return sk;
}

Eigen::Vector3d angularVel2EulerRate(const Eigen::EulerAnglesZYXd &euler, const Eigen::Vector3d &w)
{
    // see convert_angvel.m
    Eigen::Matrix3d RateMatrix;
    RateMatrix << -sin(euler.beta()), 0.0, 1.0,
        cos(euler.beta()) * sin(euler.gamma()), cos(euler.gamma()), 0.0,
        cos(euler.beta()) * cos(euler.gamma()), -sin(euler.gamma()), 0.0;

    return RateMatrix.completeOrthogonalDecomposition().solve(w);
}
