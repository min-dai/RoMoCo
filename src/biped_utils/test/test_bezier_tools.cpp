#include <gtest/gtest.h>
#include <biped_utils/bezier_tools.hpp>

using namespace bezier_tools;

TEST(BezierToolsTest, FactorialAndNchoosek)
{
   EXPECT_DOUBLE_EQ(factorial(0), 1.0);
   EXPECT_DOUBLE_EQ(factorial(1), 1.0);
   EXPECT_DOUBLE_EQ(factorial(5), 120.0);

   EXPECT_DOUBLE_EQ(nchoosek(4, 2), 6.0);
   EXPECT_DOUBLE_EQ(nchoosek(5, 0), 1.0);
   EXPECT_DOUBLE_EQ(nchoosek(5, 5), 1.0);
}

TEST(BezierToolsTest, SingleTermBezierAtEndpoints)
{
   EXPECT_DOUBLE_EQ(singleterm_bezier(3, 0, 0.0), 1.0);
   EXPECT_DOUBLE_EQ(singleterm_bezier(3, 3, 1.0), 1.0);
}

TEST(BezierToolsTest, BezierScalarCorrect)
{
   Eigen::VectorXd coeff(2);
   coeff << 0.0, 1.0;

   EXPECT_NEAR(bezier(coeff, 0.0), 0.0, 1e-9);
   EXPECT_NEAR(bezier(coeff, 1.0), 1.0, 1e-9);
   EXPECT_NEAR(bezier(coeff, 0.5), 0.5, 1e-9);
}

TEST(BezierToolsTest, BezierVectorizedCorrect)
{
   Eigen::MatrixXd coeffs(2, 2);
   coeffs << 0.0, 1.0,
       1.0, 0.0;

   Eigen::VectorXd out(2);
   bezier(coeffs, 0.0, out);
   EXPECT_NEAR(out(0), 0.0, 1e-9);
   EXPECT_NEAR(out(1), 1.0, 1e-9);

   bezier(coeffs, 1.0, out);
   EXPECT_NEAR(out(0), 1.0, 1e-9);
   EXPECT_NEAR(out(1), 0.0, 1e-9);
}

TEST(BezierToolsTest, BezierDerivatives)
{
   Eigen::VectorXd coeff(2);
   coeff << 0.0, 1.0;

   EXPECT_NEAR(dbezier(coeff, 0.5), 1.0, 1e-9);
}

TEST(BezierToolsTest, AandDerivatives)
{
   Eigen::VectorXd coeff(3);
   coeff << 0.0, 0.5, 1.0;

   double s = 0.5, sdot = 1.0;

   Eigen::MatrixXd A = A_bezier(coeff, s, sdot);
   Eigen::MatrixXd dA = dA_bezier(coeff, s, sdot);
   Eigen::MatrixXd d2A = d2A_bezier(coeff, s, sdot);

   EXPECT_EQ(A.rows(), 1);
   EXPECT_EQ(A.cols(), coeff.size());

   EXPECT_EQ(dA.rows(), 1);
   EXPECT_EQ(dA.cols(), coeff.size());

   EXPECT_EQ(d2A.rows(), 1);
   EXPECT_EQ(d2A.cols(), coeff.size());
}
