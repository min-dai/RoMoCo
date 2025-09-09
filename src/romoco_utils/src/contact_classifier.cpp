
#include <romoco_utils/contact_classifier.hpp>
#include <iostream>


ContactClassifier::ContactClassifier(const std::string &config_folder)
{
    
    Reconfigure(config_folder);
}



void ContactClassifier::Reconfigure(const std::string &config_folder)
{
    std::string config_file = config_folder + "/interface_config.yaml";
    config.yaml_parser.Init(config_file);
    config.Init();
    LowPassLeft.Reconfigure(config.dt, config.lowpass_dt_cutoff);
    LowPassRight.Reconfigure(config.dt, config.lowpass_dt_cutoff);
    grf_ = Eigen::VectorXd::Zero(6);
}

void ContactClassifier::Config::Init()
{
    dt = yaml_parser.get_double("dt");
    lowpass_dt_cutoff = yaml_parser.get_double("contact_kf/contact_classifier/lowpass_dt_cutoff");
    linear_lb = yaml_parser.get_double("contact_kf/contact_classifier/linear_lb");
    linear_ub = yaml_parser.get_double("contact_kf/contact_classifier/linear_ub");

}



ContactClassifierOutput ContactClassifier::Update(const ContactClassifierInput &input)
{
    ContactClassifierOutput output;
    // Compute the quasi-static grf estimate in world frame using statics
    // torque = J^T * F, assuming J in world frame, F is also in world frame
    grf_.segment(0, 3) = -(input.Jleft_active.transpose()).completeOrthogonalDecomposition().solve(input.torque_left);
    grf_.segment(3, 3) = -(input.Jright_active.transpose()).completeOrthogonalDecomposition().solve(input.torque_right);

    std::cout << "grf: " << grf_.transpose() << std::endl;

    // Update vertical grf lowpass
    LowPassLeft.Update(grf_(2));
    LowPassRight.Update(grf_(5));

    // Linear classifier
    output.left_contact_prob = std::clamp((LowPassLeft.getValue() - config.linear_lb) / (config.linear_ub - config.linear_lb), 0.0, 1.0);
    output.right_contact_prob = std::clamp((LowPassRight.getValue() - config.linear_lb) / (config.linear_ub - config.linear_lb), 0.0, 1.0);
    return output;
}
