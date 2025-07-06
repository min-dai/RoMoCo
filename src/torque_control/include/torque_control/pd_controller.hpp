#ifndef PD_CONTROLLER_HPP
#define PD_CONTROLLER_HPP

#include <Eigen/Dense>

class PDController
{
public:
    PDController();
    PDController(const Eigen::VectorXd &Kp, const Eigen::VectorXd &Kd);

    void reconfigure(const Eigen::VectorXd &Kp, const Eigen::VectorXd &Kd);

    Eigen::VectorXd compute(const Eigen::VectorXd &q_desired, const Eigen::VectorXd &dq_desired, const Eigen::VectorXd &q_actual, const Eigen::VectorXd &dq_actual) const;

private:
    bool is_initialized_;

    Eigen::VectorXd Kp_;
    Eigen::VectorXd Kd_;
};

#endif // PD_CONTROLLER_HPP