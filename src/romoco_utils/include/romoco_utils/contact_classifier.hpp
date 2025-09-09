
#ifndef CONTACT_CLASSIFIER_HPP
#define CONTACT_CLASSIFIER_HPP

#include <romoco_utils/filters.hpp>
#include <Eigen/Dense>
#include "romoco_utils/yaml_parser.hpp"

struct ContactClassifierOutput
{
    double left_contact_prob = 0.0;
    double right_contact_prob = 0.0;
};
struct ContactClassifierInput
{
    Eigen::MatrixXd Jleft_active;
    Eigen::MatrixXd Jright_active;
    Eigen::VectorXd torque_left;
    Eigen::VectorXd torque_right;

    void ResizeAll(int n_left, int n_right)
    {
        Jleft_active.resize(3, n_left);
        Jright_active.resize(3, n_right);
        torque_left.resize(n_left);
        torque_right.resize(n_right);
    }
    friend std::ostream &operator<<(std::ostream &os, const ContactClassifierInput &input)
    {
        os << "Jleft_active: " << input.Jleft_active << "\n";
        os << "Jright_active: " << input.Jright_active << "\n";
        os << "torque_left: " << input.torque_left << "\n";
        os << "torque_right: " << input.torque_right << "\n";
        return os;
    }
};

class ContactClassifier
{

public:
    ContactClassifier() = default;
    explicit ContactClassifier(const std::string &config_folder);
    ~ContactClassifier() = default;

    ContactClassifierOutput Update(const ContactClassifierInput &input);
    void Reconfigure(const std::string &config_folder);

private:
    control_utilities::LowPassFilter LowPassLeft = control_utilities::LowPassFilter(NAN, NAN);
    control_utilities::LowPassFilter LowPassRight = control_utilities::LowPassFilter(NAN, NAN);

    struct Config
    {
        double dt;
        double lowpass_dt_cutoff = 0.005;
        double linear_lb = 60.0;
        double linear_ub = 110.0;

        void Init();
        YAMLParser yaml_parser;

    } config;

    Eigen::VectorXd grf_;
};

#endif // CONTACT_CLASSIFIER_HPP
