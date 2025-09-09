#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "romoco_state_machine/basic_state_machine.hpp"

#include "g1_model_leg.hpp"

#include "romoco_control/torque_solver_base.hpp"
#include "romoco_core/output_base.hpp"

#include "romoco_screen_radio/radio_subscriber.hpp"
#include "romoco_screen_radio/screen_radio_conversion.hpp"

#include <cstdlib> // For getenv()

// need to set up sim
#include "g1_mujoco_interface.hpp"



VectorXd predefined_radio(double t)
{
    VectorXd radio = VectorXd::Zero(10);
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
    auto node = std::make_shared<RadioSubscriber>("g1_sim_node");


    std::string home = std::string(getenv("HOME"));

    std::string package_folder = ament_index_cpp::get_package_share_directory("g1_stack");
    std::string config_folder = package_folder + "/config_18dof";

    std::string log_path = home + "/ROBOTLOG/G1";

    std::string config_file = config_folder + "/interface_config.yaml";
    YAMLParser yaml_parser(config_file);
    std::string urdf_name = yaml_parser.get_string("urdf_name");
    std::cout << "Using URDF: " << urdf_name << std::endl;

    std::string urdf_path = package_folder + "/model_files/" + urdf_name;
    std::vector<std::string> locked_encoder_names = yaml_parser.get_string_vector("locked_encoder_names");

    VectorXd locked_joints_q = yaml_parser.get_VectorXd("qdes_locked_joints");

    std::shared_ptr<RobotBasePinocchio> robot_ptr = std::make_shared<G1ModelLeg>(urdf_path, locked_encoder_names, locked_joints_q);
    std::shared_ptr<OutputBase> output;
    std::unique_ptr<TorqueSolverBase> torque_solver;

    auto getLegModel = [](const VectorXd &q)
    { return q.head(18); };
    auto getUpper = [](const VectorXd &q)
    { return q.segment(18, 17); }; // assuming 17 upper joints

    // auto getUpper = [n_locked_joints](const VectorXd& q) {
    //     return q.tail(n_locked_joints);
    // };

    //with estimation
    std::unique_ptr<RobotBasePinocchio> robot_ptr_estimation = std::make_unique<G1ModelLeg>(urdf_path, locked_encoder_names, locked_joints_q);

    std::unique_ptr<G1MujocoInterface> mujocosim = std::make_unique<G1MujocoInterface>(config_folder, log_path, std::move(robot_ptr_estimation));


    // //no estimation
    // std::unique_ptr<G1MujocoInterface> mujocosim = std::make_unique<G1MujocoInterface>(config_folder, log_path);

    BasicStateMachine state_machine(config_folder, log_path, robot_ptr, std::move(mujocosim));

    double t_sim=0;
    VectorXd fake_radio;


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
