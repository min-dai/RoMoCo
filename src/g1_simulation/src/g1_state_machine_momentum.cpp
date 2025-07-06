#include "biped_state_machine/basic_state_machine.hpp"

#include "g1_model.hpp"

#include "torque_control/torque_solver_base.hpp"
#include "biped_core/output_base.hpp"

#include "biped_command/radio_subscriber.hpp"

#include <cstdlib> // For getenv()

// need to set up sim
#include "g1_mujoco_sim.hpp"

#include <ros/package.h>

VectorXd predefined_radio(double t)
{
    VectorXd radio = VectorXd::Zero(10);
    if (t>-1.0){
        radio(Radio::SB) = 1;

    }
    if (t>5.0){
        radio(Radio::LV) = 1;
    }

    return radio;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "g1_mujoco_node");
    ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("radio_slider_values", 10, sliderCallback);

    RadioSubscriber radio_subscriber(nh, "radio_slider_values");

    std::string home = std::string(getenv("HOME"));

    std::string package_folder = ros::package::getPath("g1_simulation");
    std::string config_folder = package_folder + "/config_29dof";
    std::string log_path = home + "/ROBOTLOG/G1";

    std::string mujoco_config_file = config_folder + "/mujoco_config.yaml";
    YAMLParser yaml_parser(mujoco_config_file);
    std::string urdf_name = yaml_parser.get_string("urdf_name");
    std::string urdf_path = package_folder + "/model_files/" + urdf_name;
    std::vector<std::string> locked_joints_names = yaml_parser.get_string_vector("locked_joints_names");
    VectorXd locked_joints_q = yaml_parser.get_VectorXd("qdes_locked_joints");

    std::shared_ptr<RobotBasePinocchio> robot_ptr = std::make_shared<G1Model>(urdf_path, locked_joints_names, locked_joints_q);
    std::shared_ptr<OutputBase> output;
    std::unique_ptr<TorqueSolverBase> torque_solver;

    //from mujoco sim q to pinocchio q, without waist locked
    auto getLegModel = [](const VectorXd& q) {
        VectorXd qmodel(29);
        qmodel << q.head(25), q.segment(28, 4);
        return qmodel;
    };
    auto getUpper = [](const VectorXd& q) {
        VectorXd qupper(6);
        qupper << q.segment(25, 3), q.segment(32,3);
        return qupper;
    };


    std::unique_ptr<G1MujocoSim> mujocosim = std::make_unique<G1MujocoSim>(config_folder);

    BasicStateMachine state_machine(config_folder, log_path, robot_ptr, std::move(mujocosim));

    double t_sim = 0.0;
    VectorXd fake_radio;
    int mode_command = 0;

    while (ros::ok())
    {
        ros::spinOnce();

        // int mode_command = radio_subscriber.mode_command();
        // VectorXd fake_radio = radio_subscriber.fake_radio();

        fake_radio = predefined_radio(t_sim);
        mode_command = fake_radio(0);

        t_sim = state_machine.Update(mode_command, fake_radio, robot_ptr, output, torque_solver, getLegModel, getUpper);
    }

    state_machine.Close();

    return 0;
}
