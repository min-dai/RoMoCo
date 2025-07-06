

#ifndef BEZIER_TOOLS_HPP
#define BEZIER_TOOLS_HPP

#include <Eigen/Dense>

namespace bezier_tools
{
    double singleterm_bezier(int m, int k, double s);
    double bezier(const Eigen::VectorXd &coeff, double s);
    void bezier(const Eigen::MatrixXd &coeffs, double s, Eigen::VectorXd &out);
    double dbezier(const Eigen::VectorXd &coeff, double s);
    void dbezier(const Eigen::MatrixXd &coeffs, double s, Eigen::VectorXd &out);
    double d2bezier(const Eigen::VectorXd &coeff, double s);
    void d2bezier(const Eigen::MatrixXd &coeffs, double s, Eigen::VectorXd &out);

    // time derivatives
    double dtime2Bezier(const Eigen::VectorXd &coeff, double s, double sdot);
    double dtimeBezier(const Eigen::VectorXd &coeff, double s, double sdot);

    double factorial(int n);
    double nchoosek(int n, int k);

    Eigen::MatrixXd A_bezier(const Eigen::VectorXd &coeff, double s, double sdot);
    Eigen::MatrixXd dA_bezier(const Eigen::VectorXd &coeff, double s, double sdot);
    Eigen::MatrixXd d2A_bezier(const Eigen::VectorXd &coeff, double s, double sdot);

}

#endif // BEZIER_TOOLS_HPP
