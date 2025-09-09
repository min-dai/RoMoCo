#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "romoco_state_machine/basic_state_machine.hpp"

#include "cassie_model.hpp"

#include "romoco_control/torque_solver_base.hpp"
#include "romoco_core/output_base.hpp"

#include "romoco_screen_radio/radio_subscriber.hpp"
#include "romoco_screen_radio/screen_radio_conversion.hpp"
#include <cstdlib> // For getenv()

// need to set up sim
#include "cassie_mujoco_interface.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadioSubscriber>("cassie_sim_node");

    std::string home = std::string(getenv("HOME"));

    std::string package_folder = ament_index_cpp::get_package_share_directory("cassie_stack");

    std::string config_folder = package_folder + "/config";
    std::string log_path = home + "/ROBOTLOG/Cassie";

    std::string config_file = config_folder + "/interface_config.yaml";
    YAMLParser yaml_parser(config_file);
    std::string urdf_name = yaml_parser.get_string("urdf_name");
    std::string urdf_path = package_folder + "/model_files/" + urdf_name;

    std::shared_ptr<RobotBasePinocchio> robot_ptr = std::make_shared<CassieModel>(urdf_path);
    std::shared_ptr<OutputBase> output;
    std::unique_ptr<TorqueSolverBase> torque_solver;

    auto getLegModel = [](const VectorXd &q)
    { return q.head(18); };
    auto getUpper = [](const VectorXd &q)
    { return VectorXd::Zero(0); };

    // with estimation
    std::unique_ptr<RobotBasePinocchio> robot_ptr_estimation = std::make_unique<CassieModel>(urdf_path);

    std::unique_ptr<CassieMujocoInterface> mujocosim = std::make_unique<CassieMujocoInterface>(config_folder, log_path, std::move(robot_ptr_estimation));

    // //no estimation
    // std::unique_ptr<CassieMujocoInterface> mujocosim = std::make_unique<CassieMujocoInterface>(config_folder, log_path);

    BasicStateMachine state_machine(config_folder, log_path, robot_ptr, std::move(mujocosim));

    double t_sim = 0.0;

    VectorXd fake_radio;

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        fake_radio = node->fake_radio();
        DesiredCommand command = ConvertScreenRadioToDesiredCommand(fake_radio);

        t_sim = state_machine.Update(command, robot_ptr, output, torque_solver, getLegModel, getUpper);
    }

    state_machine.Close();
    rclcpp::shutdown();
    return 0;
}
