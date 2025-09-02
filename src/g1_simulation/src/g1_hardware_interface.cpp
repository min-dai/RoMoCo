#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "biped_state_machine/basic_state_machine.hpp"

#include "g1_model_leg.hpp"

#include "torque_control/torque_solver_base.hpp"
#include "biped_core/output_base.hpp"

#include "screen_radio/radio_subscriber.hpp"

#include <cstdlib> // For getenv()

// need to set up sim
#include "g1_mujoco_sim.hpp"

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadioSubscriber>("g1_interface_node");


    std::string home = std::string(getenv("HOME"));

    std::string package_folder = ament_index_cpp::get_package_share_directory("g1_simulation");
    std::string config_folder = package_folder + "/config_18dof";

    std::string log_path = home + "/ROBOTLOG/G1";

    std::string mujoco_config_file = config_folder + "/mujoco_config.yaml";
    YAMLParser yaml_parser(mujoco_config_file);
    std::string urdf_name = yaml_parser.get_string("urdf_name");


    std::string urdf_path = package_folder + "/model_files/" + urdf_name;
    std::vector<std::string> locked_encoder_names = yaml_parser.get_string_vector("locked_encoder_names");
    int n_locked_joints = locked_encoder_names.size();
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

    std::unique_ptr<G1MujocoSim> mujocosim = std::make_unique<G1MujocoSim>(config_folder);

    BasicStateMachine state_machine(config_folder, log_path, robot_ptr, std::move(mujocosim));

    double t_sim = 0.0;
    VectorXd fake_radio;
    int mode_command = 0;

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

      //   // int mode_command = node->mode_command();
      //   // VectorXd fake_radio = node->fake_radio();

      //   fake_radio = predefined_radio(t_sim);
      //   mode_command = fake_radio(0);

      //   t_sim = state_machine.Update(mode_command, fake_radio, robot_ptr, output, torque_solver, getLegModel, getUpper);
    }

    rclcpp::shutdown();
    return 0;
}
