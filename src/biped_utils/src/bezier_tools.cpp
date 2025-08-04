

#include <biped_utils/bezier_tools.hpp>

namespace bezier_tools
{

    double factorial(int n)
    {
        double out = 1.;
        for (int i = 2; i <= n; ++i)
            out *= double(i);
        return out;
    }

    double nchoosek(int n, int k)
    {
        return factorial(n) / double(factorial(k) * factorial(n - k));
    }

    double singleterm_bezier(int m, int k, double s)
    {
        if (k == 0)
            return nchoosek(m, k) * std::pow(1 - s, m - k);
        else if (k == m)
            return nchoosek(m, k) * std::pow(s, k);
        else
            return nchoosek(m, k) * std::pow(s, k) * std::pow(1 - s, m - k);
    }

    double bezier(const Eigen::VectorXd &coeff, double s)
    {
        int m = coeff.size() - 1;
        double fcn = 0.;
        for (int k = 0; k <= m; ++k)
        {
            fcn += coeff(k) * singleterm_bezier(m, k, s);
        }
        return fcn;
    }

    void bezier(const Eigen::MatrixXd &coeffs, double s, Eigen::VectorXd &out)
    {
        for (int i = 0; i < coeffs.rows(); ++i)
            out(i) = bezier(coeffs.row(i), s);
    }

    void diff_coeff(const Eigen::VectorXd &coeff, Eigen::VectorXd &dcoeff)
    {
        int m = coeff.size() - 1;
        Eigen::MatrixXd A(m, m + 1);
        A.setZero();

        for (int i = 0; i < m; ++i)
        {
            A(i, i) = -(m - i) * nchoosek(m, i) / nchoosek(m - 1, i);
            A(i, i + 1) = (i + 1) * nchoosek(m, i + 1) / nchoosek(m - 1, i);
        }
        A(m - 1, m) = m * nchoosek(m, m);
        dcoeff = A * coeff;
    }

    double dbezier(const Eigen::VectorXd &coeff, double s)
    {
        Eigen::VectorXd dcoeff;
        dcoeff.resizeLike(coeff);
        diff_coeff(coeff, dcoeff);
        return bezier(dcoeff, s);
    }

    void dbezier(const Eigen::MatrixXd &coeffs, double s, Eigen::VectorXd &out)
    {
        for (int i = 0; i < coeffs.rows(); ++i)
            out(i) = dbezier(coeffs.row(i), s);
    }

    double d2bezier(const Eigen::VectorXd &coeff, double s)
    {
        Eigen::VectorXd dcoeff, d2coeff;
        dcoeff.resizeLike(coeff);
        d2coeff.resizeLike(coeff);
        diff_coeff(coeff, dcoeff);
        diff_coeff(dcoeff, d2coeff);
        return bezier(d2coeff, s);
    }

    void d2bezier(const Eigen::MatrixXd &coeffs, double s, Eigen::VectorXd &out)
    {
        for (int i = 0; i < coeffs.rows(); ++i)
            out(i) = d2bezier(coeffs.row(i), s);
    }

    double dtimeBezier(const Eigen::VectorXd &coeff, double s, double sdot)
    {
        double out;
        Eigen::VectorXd dcoeff;
        dcoeff.resizeLike(coeff);
        diff_coeff(coeff, dcoeff);
        out = bezier(dcoeff, s) * sdot;
        return out;
    }
    double dtime2Bezier(const Eigen::VectorXd &coeff, double s, double sdot)
    {
        double out;
        Eigen::VectorXd dcoeff, d2coeff;
        dcoeff.resizeLike(coeff);
        d2coeff.resizeLike(coeff);
        diff_coeff(coeff, dcoeff);
        diff_coeff(dcoeff, d2coeff);
        out = bezier(d2coeff, s) * pow(sdot, 2);
        return out;
    }

    Eigen::MatrixXd A_bezier(const Eigen::VectorXd &coeff, double s)
    {
        Eigen::MatrixXd A;
        int ncoeff = coeff.size();
        int M = ncoeff - 1;
        A.resize(1, ncoeff);
        A.setZero();
        for (int j = 0; j < ncoeff; j++)
        {
            A(0, j) = nchoosek(M, j) * pow(s, j) * pow(1 - s, M - j);
        }
        return A;
    };

    Eigen::MatrixXd dA_bezier(const Eigen::VectorXd &coeff, double s, double sdot)
    {
        Eigen::MatrixXd A, A_vec, A_mat;
        int ncoeff = coeff.size();
        int M = ncoeff - 1;

        A_vec = Eigen::MatrixXd::Zero(1, ncoeff - 1);
        A_mat = Eigen::MatrixXd::Zero(ncoeff - 1, ncoeff);
        for (int j = 0; j < M; j++)
        {
            A_vec(0, j) = factorial(M) / double(factorial(j)) / double(factorial(M - j - 1)) * pow(s, j) * pow(1 - s, M - j - 1);
            A_mat(j, j) = -1.;
            A_mat(j, j + 1) = 1.;
        }
        A = sdot * A_vec * A_mat;
        return A;
    };

    Eigen::MatrixXd d2A_bezier(const Eigen::VectorXd &coeff, double s, double sdot)
    {
        Eigen::MatrixXd A, A_vec, A_mat;
        int ncoeff = coeff.size();
        A_vec = Eigen::MatrixXd::Zero(1, ncoeff - 2);
        A_mat = Eigen::MatrixXd::Zero(ncoeff - 2, ncoeff);
        int M = ncoeff - 1;

        for (int j = 0; j < ncoeff - 2; j++)
        {
            A_vec(0, j) = factorial(ncoeff - 1) / double(factorial(j) * factorial(M - j - 2)) * pow(s, j) * pow(1 - s, M - j - 2);
            A_mat(j, j) = 1.;
            A_mat(j, j + 1) = -2.;
            A_mat(j, j + 2) = 1.;
        }
        A = pow(sdot, 2) * A_vec * A_mat;
        return A;
    };

} // namespace bezier_tools
