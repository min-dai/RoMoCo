
#include <biped_utils/contact_classifier.hpp>

using namespace std;

ContactClassifier::ContactClassifier(double dt)
{
    config.Init(dt);
    Reconfigure();
}

void ContactClassifier::Reconfigure()
{
    LowPassLeft.Reconfigure(config.dt, config.lowpass_dt_cutoff);
    LowPassRight.Reconfigure(config.dt, config.lowpass_dt_cutoff);
    grf_ = Eigen::VectorXd::Zero(6);
}
void ContactClassifier::Reconfigure(double dt, double lowpass_dt_cutoff, double linear_lb, double linear_ub)
{
    config.Init(dt, lowpass_dt_cutoff, linear_lb, linear_ub);
    Reconfigure();
}

ContactClassifierOutput ContactClassifier::Update(const ContactClassifierInput &input)
{
    ContactClassifierOutput output;
    // Compute the quasi-static grf estimate in world frame using statics
    // torque = J^T * F, assuming J in world frame, F is also in world frame
    grf_.segment(0, 3) = -(input.Jleft_active.transpose()).completeOrthogonalDecomposition().solve(input.torque_left);
    grf_.segment(3, 3) = -(input.Jright_active.transpose()).completeOrthogonalDecomposition().solve(input.torque_right);
    // Update vertical grf lowpass
    LowPassLeft.Update(grf_(2));
    LowPassRight.Update(grf_(5));

    // Linear classifier
    output.left_contact_prob = std::clamp((LowPassLeft.getValue() - config.linear_lb) / (config.linear_ub - config.linear_lb), 0.0, 1.0);
    output.right_contact_prob = std::clamp((LowPassRight.getValue() - config.linear_lb) / (config.linear_ub - config.linear_lb), 0.0, 1.0);

    return output;
}
