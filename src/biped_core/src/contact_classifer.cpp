
#include <biped_estimation/contact_classifier.hpp>


using namespace std;


ContactClassifier::ContactClassifier(std::shared_ptr<RobotBasePinocchio> robot, double dt)
    : LowPassLeft(dt, 1 / 300), LowPassRight(dt, 1 / 300)
{
    this->config.paramChecker.init(nh.getNamespace() + "/contact_classifier");

    // Pointer to the robot
    this->robot = &robot;

    this->config.init();


    this->grf.resize(6);
}


void ContactClassifier::Config::init() {
    this->dt = 0.0005;
    this->lowpass_dt_cutoff = 1 / 300.;
}

void ContactClassifier::Config::reconfigure() {

    paramChecker.checkAndUpdate("/cassie/dt", this->dt);
    paramChecker.checkAndUpdate("linear_lb", this->linear_lb);
    paramChecker.checkAndUpdate("linear_ub", this->linear_ub);
    paramChecker.checkAndUpdate("lowpass_dt_cutoff", this->lowpass_dt_cutoff);
}

void ContactClassifier::reconfigure() {
    this->config.reconfigure();
    this->LowPassLeft.reconfigure(this->config.dt, this->config.lowpass_dt_cutoff);
    this->LowPassRight.reconfigure(this->config.dt, this->config.lowpass_dt_cutoff);
}



void ContactClassifier::update() {
    MatrixXd Jleft(3, 5), Jright(3, 5);

    this->robot->kinematics.computeConstrainedToeJacobian(this->robot->q, Jleft, Jright);

    // Compute the quasi-static grf estimate in world frame using statics
    // tau = J^T * F, assuming J in world frame, F is also in world frame
    this->grf.segment(0, 3) = -(Jleft.transpose()).completeOrthogonalDecomposition().solve(
        this->robot->torque.segment(0, 5));
    this->grf.segment(3, 3) = -(Jright.transpose()).completeOrthogonalDecomposition().solve(
        this->robot->torque.segment(5, 5));


    // Update vertical grf lowpass
    this->LowPassLeft.update(this->grf(2));
    this->LowPassRight.update(this->grf(5));

    // Linear classifier
    this->robot->leftContact = control_utilities::clamp((this->LowPassLeft.getValue() - this->config.linear_lb) / (this->config.linear_ub - this->config.linear_lb), 0, 1);
    this->robot->rightContact = control_utilities::clamp((this->LowPassRight.getValue() - this->config.linear_lb) / (this->config.linear_ub - this->config.linear_lb), 0, 1);

    this->robot->GRF = grf;

}
