
#include <Eigen/Dense>


#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <Clarabel>

using namespace clarabel;
using namespace std;
using namespace Eigen;

class TestClarabel : public ::testing::Test
{
public:
    TestClarabel()
    {
    }
    static constexpr double tol = 1e-5;
};

TEST_F(TestClarabel, solve)
{

    /* From dense matrix:
     * [[6., 0.],
     *  [0., 4.]]
     */
    MatrixXd P_dense(2, 2);
    P_dense << 6., 0.,
        0., 4.;

    SparseMatrix<double> P = P_dense.sparseView();

    P.makeCompressed();

    Vector<double, 2> q = {-1., -4.};

    MatrixXd A_dense(5, 2);
    A_dense << 1., -2., // <-- LHS of equality constraint (lower bound)
        1., 0.,         // <-- LHS of inequality constraint (upper bound)
        0., 1.,         // <-- LHS of inequality constraint (upper bound)
        -1., 0.,        // <-- LHS of inequality constraint (lower bound)
        0., -1.;        // <-- LHS of inequality constraint (lower bound)

    SparseMatrix<double> A = A_dense.sparseView();
    A.makeCompressed();

    Vector<double, 5> b = {0., 1., 1., 1., 1.};

    vector<SupportedConeT<double>> cones{
        ZeroConeT<double>(1),
        NonnegativeConeT<double>(4),
    };

    // Settings
    DefaultSettings<double> settings = DefaultSettings<double>::default_settings();
    settings.presolve_enable = false;
    // Build solver
    DefaultSolver<double> solver(P, q, A, b, cones, settings);

    // Solve
    solver.solve();

    P_dense << 16., 0.,
        0., 4.;

    SparseMatrix<double> P2 = P_dense.sparseView();

    A_dense << 1., -2., // <-- LHS of equality constraint (lower bound)
        1., 1.,         // <-- LHS of inequality constraint (upper bound)
        0., 1.,         // <-- LHS of inequality constraint (upper bound)
        -1., 0.,        // <-- LHS of inequality constraint (lower bound)
        0., -1.;        // <-- LHS of inequality constraint (lower bound)

    SparseMatrix<double> A2 = A_dense.sparseView();

    // revised original solver
    solver.update_P(P2);
    solver.update_A(A2);
    b(0) = 1;
    solver.update_b(b);
    q(0) = 0;
    solver.update_q(q);
    solver.solve();

    // Get solution
    DefaultSolution<double> solution = solver.solution();
    VectorXd solutionVector = solution.x;

    cout << "Solution: " << solutionVector.transpose() << endl;
    // utils::print_solution(solution);

    cout << "A*x = " << (A_dense * solutionVector).transpose() << endl;
    cout << "b   = " << b.transpose() << endl;
}