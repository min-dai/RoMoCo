#ifndef BASIC_CONTROLLER_STATE_MACHINE_HPP
#define BASIC_CONTROLLER_STATE_MACHINE_HPP

#include "romoco_core/robot_base_pinocchio.hpp"

#include "romoco_core/mujoco_interface_base.hpp"

#include "romoco_utils/pd_controller.hpp"
#include "romoco_control/torque_solver_base.hpp"
#include "romoco_core/output_base.hpp"

#include "romoco_core/biped_commands.hpp"
#include "romoco_core/biped_motor_commands.hpp"
#include "romoco_utils/simple_timer.hpp"
#include <fstream> // For std::fstream
#include <string>  // For std::string

namespace romoco
{
   /**
    * @class BasicControllerStateMachine
    * @ingroup group_interface
    * @brief A basic controller state machine for controlling a biped robot.
    * @details This class implements a simple state machine for controlling a biped robot using different controllers based on the desired mode of operation.
    * It supports modes such as Null, Standing, and Walking, and switches between different controllers accordingly.
    * @param config_folder The folder containing configuration files for the controllers for each behavior mode.
    * @param log_path The path to save log files.
    * @param robot_ptr A shared pointer to the robot model (RobotBasePinocchio).
    */
   class BasicControllerStateMachine
   {
   public:
      BasicControllerStateMachine() = default;
      BasicControllerStateMachine(const std::string &config_folder, const std::string &log_path, std::shared_ptr<romoco::robot::RobotBasePinocchio> robot_ptr);
      virtual ~BasicControllerStateMachine();
      /**
       * @brief Update the motor control commands based on the desired command and current robot state.
       * @param command The desired command parsed from user inputs, e.g., from radio sliders, or connected radio hardware.
       * @param robot_ptr A shared pointer to the robot model (RobotBasePinocchio).
       * @param output A shared pointer to the current output controller (OutputBase).
       * @param torque_solver A unique pointer to the current torque solver (TorqueSolverBase).
       * @param q_loco The current joint positions of the robot for locomotion control.
       * @param dq_loco The current joint velocities of the robot for locomotion control.
       * @return BipedMotorCommands The computed motor commands for the robot.
       * @details This method updates the motor control commands based on the desired command and the current robot state.
       * It selects the appropriate controller based on the desired mode and computes the motor commands accordingly.
       * It also logs relevant data to .bin for analysis.
       */

      BipedMotorCommands UpdateControl(const DesiredCommand &command,
                                       std::shared_ptr<romoco::robot::RobotBasePinocchio> robot_ptr,
                                       std::shared_ptr<OutputBase> &output,
                                       std::unique_ptr<TorqueSolverBase> &torque_solver,
                                       Eigen::VectorXd &q_loco, Eigen::VectorXd &dq_loco);

      void Init(const std::string &config_folder, const std::string &log_path, std::shared_ptr<romoco::robot::RobotBasePinocchio> robot_ptr);

      void Close();

   private:
      Eigen::VectorXf CollectLog(const double t, const std::vector<Eigen::VectorXd> &vectors);

      void SelectControllers(Mode mode, const std::string &config_file, std::shared_ptr<romoco::robot::RobotBasePinocchio> robot_ptr, std::shared_ptr<OutputBase> &output, std::unique_ptr<TorqueSolverBase> &torque_solver);

      // path
      std::string config_folder_;
      std::string log_path_;

      Mode cur_mode_ = Mode::Null;
      Eigen::VectorXd locomotion_input_; // input for locomotion controller

      // for Log files
      std::fstream logFile_;
      std::string logFilePath_;

      SimpleTimer timer_;

      BipedMotorCommands motor_commands_;
   };

} // namespace romoco

#endif // BASIC_CONTROLLER_STATE_MACHINE_HPP