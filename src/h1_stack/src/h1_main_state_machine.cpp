#include <rclcpp/rclcpp.hpp>

#include "romoco_state_machine/full_state_machine.hpp"

#include "h1_model_leg.hpp"

#include "romoco_control/torque_solver_base.hpp"
#include "romoco_core/output_base.hpp"

#include "romoco_screen_radio/radio_subscriber.hpp"
#include "romoco_screen_radio/screen_radio_conversion.hpp"

#include "romoco_ros/ros_load_config.hpp"

#include "h1_mujoco_interface.hpp"

using namespace romoco;

Eigen::VectorXd predefined_radio(double t)
{
    Eigen::VectorXd radio = Eigen::VectorXd::Zero(10);
    if (t > 1.0)
    {
        radio(ScreenRadio::Mode) = 1;
    }
    if (t > 10.0)
    {
        radio(ScreenRadio::X) = 1;
    }

    return radio;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadioSubscriber>("h1_sim_node");

    bool use_estimation = false;
    if (argc > 1)
    {
        use_estimation = std::string(argv[1]) == "true" || std::string(argv[1]) == "1";
    }

    RosLoadConfig ros_config("h1_stack", "config_18dof", "ROBOTLOG/H1");

    std::shared_ptr<romoco::robot::RobotBasePinocchio> robot_ptr = std::make_shared<romoco::robot::H1ModelLeg>(ros_config.config_folder);
    std::shared_ptr<OutputBase> output;
    std::unique_ptr<TorqueSolverBase> torque_solver;

    auto getLegModel = [](const Eigen::VectorXd &q)
    { return q.head(18); };
    auto getUpper = [](const Eigen::VectorXd &q)
    { return q.segment(18, 39); }; // assuming 39 upper joints

    std::unique_ptr<MujocoInterfaceBase> interface;

    if (use_estimation)
    {
        std::unique_ptr<romoco::robot::RobotBasePinocchio> robot_ptr_estimation = std::make_unique<romoco::robot::H1ModelLeg>(ros_config.config_folder);

        interface = std::make_unique<H1MujocoInterface>(ros_config.config_folder, ros_config.log_path, std::move(robot_ptr_estimation));
    }
    else
    {
        interface = std::make_unique<H1MujocoInterface>(ros_config.config_folder, ros_config.log_path);
    }
    BasicStateMachine state_machine(ros_config.config_folder, ros_config.log_path, robot_ptr, std::move(interface));

    double t_sim=0;
    Eigen::VectorXd fake_radio;


    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        fake_radio = node->fake_radio();

        // fake_radio = predefined_radio(t_sim);
        DesiredCommand command = ConvertScreenRadioToDesiredCommand(fake_radio);


        t_sim = state_machine.Update(command, robot_ptr, output, torque_solver, getLegModel, getUpper);
    }


    rclcpp::shutdown();
    return 0;
}
