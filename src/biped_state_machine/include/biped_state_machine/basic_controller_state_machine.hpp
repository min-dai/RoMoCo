#ifndef BASIC_STATE_MACHINE_HPP
#define BASIC_STATE_MACHINE_HPP

#include "biped_core/robot_base_pinocchio.hpp"

#include "biped_core/mujoco_sim_base.hpp"

#include "torque_control/pd_controller.hpp"
#include "torque_control/torque_solver_base.hpp"
#include "biped_core/output_base.hpp"

#include "biped_types/biped_commands.hpp"
#include "biped_types/biped_motor_commands.hpp"
#include "biped_utils/simple_timer.hpp"
#include <fstream> // For std::fstream
#include <string>  // For std::string

class BasicControllerStateMachine
{
public:
   BasicControllerStateMachine() = default;
   BasicControllerStateMachine(const std::string &config_folder, const std::string &log_path, std::shared_ptr<RobotBasePinocchio> robot_ptr);
   virtual ~BasicControllerStateMachine();

   BipedMotorCommands UpdateControl(const DesiredCommand &command,
                                    std::shared_ptr<RobotBasePinocchio> robot_ptr,
                                    std::shared_ptr<OutputBase> &output,
                                    std::unique_ptr<TorqueSolverBase> &torque_solver,
                                    Eigen::VectorXd &q_loco, Eigen::VectorXd &dq_loco);

   void Init(const std::string &config_folder, const std::string &log_path, std::shared_ptr<RobotBasePinocchio> robot_ptr);

   void Close();

private:
   Eigen::VectorXf CollectLog(const double t, const std::vector<VectorXd> &vectors);

   void SelectControllers(Mode mode, const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot_ptr, std::shared_ptr<OutputBase> &output, std::unique_ptr<TorqueSolverBase> &torque_solver);

   // path
   std::string config_folder_;
   std::string log_path_;

   Mode cur_mode_ = Mode::Null;
   Eigen::VectorXd qfull_, dqfull_;
   Eigen::VectorXd locomotion_input_; // input for locomotion controller
   Eigen::VectorXd locked_input_;     // input for locked joints PD controller
   double t_old_ = 0;

   // for Log files
   std::fstream logFile_;
   std::string logFilePath_;

   // for upper body locked joints PD control
   int n_locked_joints_ = 0;
   VectorXd q_locked_joints_des_, dq_locked_joints_des_;
   PDController locked_joints_pd_controller_;
   SimpleTimer timer_;

   BipedMotorCommands motor_commands_;
};

#endif // BASIC_STATE_MACHINE_HPP