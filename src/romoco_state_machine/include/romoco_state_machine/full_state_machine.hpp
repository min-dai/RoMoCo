#ifndef BASIC_STATE_MACHINE_HPP
#define BASIC_STATE_MACHINE_HPP

#include "romoco_core/robot_base_pinocchio.hpp"

#include "romoco_core/mujoco_interface_base.hpp"

#include "romoco_utils/pd_controller.hpp"
#include "romoco_control/torque_solver_base.hpp"
#include "romoco_core/output_base.hpp"

#include "romoco_core/biped_commands.hpp"

#include <fstream> // For std::fstream
#include <string>  // For std::string

namespace romoco
{
   /**
    * @class BasicStateMachine
    * @ingroup group_controller
    * @brief A basic state machine for controlling a biped robot.
    * @details This class implements a simple state machine with mujoco simulation interface for synchronized control of a biped robot using different controllers based on the desired mode of operation.
    * It supports modes such as Null, Standing, and Walking, and switches between different controllers accordingly.
    * @param config_folder The folder containing configuration files for the controllers for each behavior mode.
    * @param log_path The path to save log files.
    * @param robot_ptr A shared pointer to the robot model (RobotBasePinocchio).
    * @param sim A unique pointer to the simulation interface (MujocoInterfaceBase).
    */
class BasicStateMachine
{
public:
   BasicStateMachine() = default;
   BasicStateMachine(const std::string &config_folder, const std::string &log_path, std::shared_ptr<romoco::robot::RobotBasePinocchio> robot_ptr, std::unique_ptr<MujocoInterfaceBase> sim);
   virtual ~BasicStateMachine();

   double Update(const DesiredCommand &command, std::shared_ptr<romoco::robot::RobotBasePinocchio> robot_ptr, std::shared_ptr<OutputBase> &output, std::unique_ptr<TorqueSolverBase> &torque_solver, std::function<Eigen::VectorXd(const Eigen::VectorXd &)> getLegModel, std::function<Eigen::VectorXd(const Eigen::VectorXd &)> getUpper);

   void Init(const std::string &config_folder, const std::string &log_path, std::shared_ptr<romoco::robot::RobotBasePinocchio> robot_ptr);

   void Close();

private:
   Eigen::VectorXf CollectLog(const double t, const std::vector<Eigen::VectorXd> &vectors);

   void SelectControllers(Mode mode, const std::string &config_file, std::shared_ptr<romoco::robot::RobotBasePinocchio> robot_ptr, std::shared_ptr<OutputBase> &output, std::unique_ptr<TorqueSolverBase> &torque_solver);

   // path
   std::string config_folder_;
   std::string log_path_;

   Mode cur_mode_ = Mode::Null;
   Eigen::VectorXd qfull_, dqfull_;
   Eigen::VectorXd locomotion_input_; // input for locomotion controller
   Eigen::VectorXd locked_input_;     // input for locked joints PD controller
   double t_old_ = 0;
   int control_counter_ = 0;
   int control_counter_threshold_ = 1;

   BipedMotorCommands motor_commands_;

   std::unique_ptr<MujocoInterfaceBase> sim_;

   // for Log files
   std::fstream logFile_;
   std::string logFilePath_;

   // for upper body locked joints PD control
   int n_locked_joints_ = 0;
   Eigen::VectorXd q_locked_joints_des_, dq_locked_joints_des_;
   PDController locked_joints_pd_controller_;
};

} // namespace romoco

#endif // BASIC_STATE_MACHINE_HPP