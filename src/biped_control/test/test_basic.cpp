
#include <Eigen/Dense>

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <Clarabel>
#include <gtest/gtest.h>
using namespace clarabel;
using namespace std;
using namespace Eigen;

class TestBasic : public ::testing::Test
{
public:
    TestBasic()
    {
    }
    static constexpr double tol = 1e-5;
};

TEST_F(TestBasic, solve)
{
    VectorXd x = VectorXd::Zero(2);
    cout << "x size: " << x.size() << endl;

    MatrixXd A = MatrixXd::Zero(2, 3);
    cout << "A size: " << A.rows() << " x " << A.cols() << endl;

    std::vector<double> v = {1, 2, 3, 4, 5};
    cout << "v size: " << v.size() << endl;

}