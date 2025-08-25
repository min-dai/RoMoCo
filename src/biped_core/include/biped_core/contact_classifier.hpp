
#ifndef CONTACT_CLASSIFIER_HPP
#define CONTACT_CLASSIFIER_HPP

#include <biped_core/robot_base_pinocchio.hpp>
#include <biped_utils/filters.hpp>



class ContactClassifier {

public:

    VectorXd grf;

    ContactClassifier(std::shared_ptr<RobotBasePinocchio> robot, double dt);
    void update();
    void reconfigure();

private:
    control_utilities::LowPassFilter LowPassLeft;
    control_utilities::LowPassFilter LowPassRight;


    struct Config {
        double dt;
        double lowpass_dt_cutoff;
        double linear_lb;
        double linear_ub;



        void init();
        void reconfigure();
    } config;


};

#endif // CONTACT_CLASSIFIER_HPP
