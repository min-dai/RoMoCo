

#ifndef BEZIER_TOOLS_HPP
#define BEZIER_TOOLS_HPP

#include <Eigen/Dense>
namespace romoco
{
/**
 * @namespace romoco::bezier_tools
 * @ingroup group_utils
 * @brief Utilities for working with Bézier curves and their derivatives.
 */
namespace bezier_tools
{
    /**
     * @brief Compute a single Bernstein polynomial term.
     * @param m Order of the polynomial.
     * @param k Index of the basis function.
     * @param s Curve parameter in [0, 1].
     * @return Value of the Bernstein basis term.
     */
    double singleterm_bezier(int m, int k, double s);

    /**
     * @brief Evaluate a Bézier curve.
     * @ingroup group_utils
     * @details Evaluates a Bézier curve at a given parameter value.
     */
    double bezier(const Eigen::VectorXd &coeff, double s);

    /**
     * @brief Evaluate a vector-valued Bézier curve.
     * @ingroup group_utils
     * @details Evaluates a vector-valued Bézier curve at a given parameter value.
     */
    void bezier(const Eigen::MatrixXd &coeffs, double s, Eigen::VectorXd &out);

    /// @brief First derivative of a Bézier curve.
    double dbezier(const Eigen::VectorXd &coeff, double s);

    /// @brief First derivative of a vector-valued Bézier curve.
    void dbezier(const Eigen::MatrixXd &coeffs, double s, Eigen::VectorXd &out);

    /// @brief Second derivative of a Bézier curve.
    double d2bezier(const Eigen::VectorXd &coeff, double s);

    /// @brief Second derivative of a vector-valued Bézier curve.
    void d2bezier(const Eigen::MatrixXd &coeffs, double s, Eigen::VectorXd &out);


    /**
     * @brief Time derivative (first) using chain rule.
     * @ingroup group_utils
     * @details Computes the time derivative of a Bézier curve using the chain rule.
     */
    double dtimeBezier(const Eigen::VectorXd &coeff, double s, double sdot);
    /**
     * @brief Time derivative (second) using chain rule.
     * @ingroup group_utils
     * @details Computes the second time derivative of a Bézier curve using the chain rule.
     */
    double dtime2Bezier(const Eigen::VectorXd &coeff, double s, double sdot);

    /// @brief Compute factorial of n.
    double factorial(int n);

    /// @brief Binomial coefficient "n choose k".
    double nchoosek(int n, int k);

    /// @brief Construct Bézier evaluation matrix.
    Eigen::MatrixXd A_bezier(const Eigen::VectorXd &coeff, double s);

    /// @brief Construct first derivative evaluation matrix.
    Eigen::MatrixXd dA_bezier(const Eigen::VectorXd &coeff, double s, double sdot);

    /// @brief Construct second derivative evaluation matrix.
    Eigen::MatrixXd d2A_bezier(const Eigen::VectorXd &coeff, double s, double sdot);

}  // namespace bezier_tools
}  // namespace romoco
#endif // BEZIER_TOOLS_HPP
