#include "romoco_utils/pd_controller.hpp"

PDController::PDController() : is_initialized_(false) {}

// Constructs a PDController with given Kp and Kd gains.
// Kp and Kd must be the same size, otherwise std::invalid_argument is thrown.
PDController::PDController(const Eigen::VectorXd &Kp, const Eigen::VectorXd &Kd)
{
    Reconfigure(Kp, Kd);
}
PDController::PDController(const Eigen::VectorXd &Kp, const Eigen::VectorXd &Kd, const Eigen::VectorXd &q_desired, const Eigen::VectorXd &dq_desired)
{
    Reconfigure(Kp, Kd, q_desired, dq_desired);
}

void PDController::Reconfigure(const Eigen::VectorXd &Kp, const Eigen::VectorXd &Kd)
{
    // check that Kp and Kd are the same size using standard library assert
    if (Kp.size() != Kd.size())
    {
        throw std::invalid_argument("Kp and Kd must be the same size");
    }

    Kp_ = Kp;
    Kd_ = Kd;
    is_initialized_ = true;
}

void PDController::Reconfigure(const Eigen::VectorXd &Kp, const Eigen::VectorXd &Kd, const Eigen::VectorXd &q_desired, const Eigen::VectorXd &dq_desired)
{
    // check that Kp and Kd are the same size using standard library assert
    if (Kp.size() != Kd.size() || Kp.size() != q_desired.size() || Kp.size() != dq_desired.size())
    {
        throw std::invalid_argument("Kp, Kd, q_desired, and dq_desired must be the same size");
    }

    Kp_ = Kp;
    Kd_ = Kd;
    q_desired_ = q_desired;
    dq_desired_ = dq_desired;
    is_initialized_ = true;
}

Eigen::VectorXd PDController::Compute(const Eigen::VectorXd &q_desired, const Eigen::VectorXd &dq_desired, const Eigen::VectorXd &q_actual, const Eigen::VectorXd &dq_actual) const
{
    // Cannot run if not initialized
    if (!is_initialized_)
    {
        throw std::runtime_error("PDController is not initialized. Call Reconfigure() first.");
    }

    if (q_desired.size() != Kd_.size() || dq_desired.size() != Kd_.size() ||
        q_actual.size() != Kd_.size() ||
        dq_actual.size() != Kd_.size())
    {
        throw std::invalid_argument("q_desired, dq_desired, q_actual, and dq_actual must all be the same size as Kd_");
    }
    // Compute and return the vector
    return -(Kp_.cwiseProduct(q_actual - q_desired) + Kd_.cwiseProduct(dq_actual - dq_desired));
}

Eigen::VectorXd PDController::Compute(const Eigen::VectorXd &q_actual, const Eigen::VectorXd &dq_actual) const
{
    // Cannot run if not initialized
    if (!is_initialized_)
    {
        throw std::runtime_error("PDController is not initialized. Call Reconfigure() first.");
    }

    if (q_desired_.size() != Kd_.size() || dq_desired_.size() != Kd_.size() || q_actual.size() != Kd_.size() || dq_actual.size() != Kd_.size())
    {
        throw std::invalid_argument("q_desired_, dq_desired_, q_actual, and dq_actual must all be the same size as Kd_");
    }

    // Compute and return the vector
    return -(Kp_.cwiseProduct(q_actual - q_desired_) + Kd_.cwiseProduct(dq_actual - dq_desired_));
}