#include <rclcpp/rclcpp.hpp>

#include "romoco_state_machine/full_state_machine.hpp"

#include "cassie_model.hpp"

#include "romoco_control/torque_solver_base.hpp"
#include "romoco_core/output_base.hpp"

#include "romoco_screen_radio/radio_subscriber.hpp"
#include "romoco_screen_radio/screen_radio_conversion.hpp"

#include "romoco_ros/ros_load_config.hpp"

#include "cassie_mujoco_interface.hpp"

using namespace romoco;

Eigen::VectorXd predefined_radio(double t)
{
    Eigen::VectorXd radio = Eigen::VectorXd::Zero(10);
    if (t > 1.0)
    {
        radio(ScreenRadio::SB) = 1;
    }
    if (t > 10.0)
    {
        radio(ScreenRadio::LV) = 1;
    }

    return radio;
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::string config_folder_name = "config";

    RosLoadConfig ros_config("cassie_stack", config_folder_name, "ROBOTLOG/Cassie");


    auto node = std::make_shared<RadioSubscriber>("cassie_sim_node");


    std::string config_folder = ros_config.config_folder;
    std::string log_path = ros_config.log_path;



    std::shared_ptr<romoco::robot::RobotBasePinocchio> robot_ptr = std::make_shared<romoco::robot::CassieModel>(ros_config.config_folder);
    std::shared_ptr<OutputBase> output;
    std::unique_ptr<TorqueSolverBase> torque_solver;

    auto getLegModel = [](const Eigen::VectorXd &q)
    { return q.head(18); };
    auto getUpper = [](const Eigen::VectorXd &q)
    { return Eigen::VectorXd::Zero(0); };

    // with estimation
    std::unique_ptr<romoco::robot::RobotBasePinocchio> robot_ptr_estimation = std::make_unique<romoco::robot::CassieModel>(ros_config.config_folder);

    std::unique_ptr<CassieMujocoInterface> mujocosim = std::make_unique<CassieMujocoInterface>(config_folder, log_path, std::move(robot_ptr_estimation));

    // //no estimation
    // std::unique_ptr<CassieMujocoInterface> mujocosim = std::make_unique<CassieMujocoInterface>(config_folder, log_path);

    BasicStateMachine state_machine(config_folder, log_path, robot_ptr, std::move(mujocosim));

    double t_sim = 0.0;

    Eigen::VectorXd fake_radio;

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        // fake_radio = node->fake_radio();

        fake_radio = predefined_radio(t_sim);
        DesiredCommand command = ConvertScreenRadioToDesiredCommand(fake_radio);

        t_sim = state_machine.Update(command, robot_ptr, output, torque_solver, getLegModel, getUpper);
    }

    state_machine.Close();
    rclcpp::shutdown();
    return 0;
}
