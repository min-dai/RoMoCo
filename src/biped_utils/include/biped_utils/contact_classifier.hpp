
#ifndef CONTACT_CLASSIFIER_HPP
#define CONTACT_CLASSIFIER_HPP

#include <biped_utils/filters.hpp>
#include <Eigen/Dense>

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
    explicit ContactClassifier(double dt);
    ~ContactClassifier() = default;

    ContactClassifierOutput Update(const ContactClassifierInput &input);
    void Reconfigure(double dt);
    void Reconfigure(double dt, double lowpass_dt_cutoff, double linear_lb, double linear_ub);

private:
    control_utilities::LowPassFilter LowPassLeft = control_utilities::LowPassFilter(NAN, NAN);
    control_utilities::LowPassFilter LowPassRight = control_utilities::LowPassFilter(NAN, NAN);

    struct Config
    {
        double dt;
        double lowpass_dt_cutoff = 0.005;
        double linear_lb = 60.0;
        double linear_ub = 110.0;

        void Init(double dt){
            this->dt = dt;
        };
        void Init(double dt, double lowpass_dt_cutoff, double linear_lb, double linear_ub) {
            this->dt = dt;
            this->lowpass_dt_cutoff = lowpass_dt_cutoff;
            this->linear_lb = linear_lb;
            this->linear_ub = linear_ub;
        }
    } config;

    Eigen::VectorXd grf_;
};

#endif // CONTACT_CLASSIFIER_HPP
