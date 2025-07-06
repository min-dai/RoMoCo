#ifndef BASIC_STATE_MACHINE_HPP
#define BASIC_STATE_MACHINE_HPP

#include "biped_core/robot_base_pinocchio.hpp"

#include "biped_core/mujoco_sim_base.hpp"

#include "torque_control/pd_controller.hpp"
#include "torque_control/torque_solver_base.hpp"
#include "biped_core/output_base.hpp"

#include "biped_command/radio_slider_map.hpp"

#include <fstream> // For std::fstream
#include <string>  // For std::string

class BasicStateMachine
{
public:
   BasicStateMachine() = default;
   BasicStateMachine(const std::string &config_folder, const std::string &log_path, std::shared_ptr<RobotBasePinocchio> robot_ptr, std::unique_ptr<MujocoSimBase> sim);
   virtual ~BasicStateMachine();

   double Update(const int mode_command, const VectorXd &fake_radio, std::shared_ptr<RobotBasePinocchio> robot_ptr, std::shared_ptr<OutputBase> &output, std::unique_ptr<TorqueSolverBase> &torque_solver, std::function<Eigen::VectorXd(const Eigen::VectorXd &)> getLegModel, std::function<Eigen::VectorXd(const Eigen::VectorXd &)> getUpper);

   void Init(const std::string &config_folder, const std::string &log_path, std::shared_ptr<RobotBasePinocchio> robot_ptr);

   void Close();

private:
   Eigen::VectorXf CollectLog(const double t, const std::vector<VectorXd> &vectors);

   void SelectControllers(int mode, const std::string &config_file, std::shared_ptr<RobotBasePinocchio> robot_ptr, std::shared_ptr<OutputBase> &output, std::unique_ptr<TorqueSolverBase> &torque_solver);

   // path
   std::string config_folder_;
   std::string log_path_;

   int cur_mode_ = Radio::Null;
   Eigen::VectorXd qfull_, dqfull_;
   Eigen::VectorXd locomotion_input_; // input for locomotion controller
   Eigen::VectorXd locked_input_;     // input for locked joints PD controller
   double t_old_ = 0;
   int control_counter_ = 0;
   int control_counter_threshold_ = 1;

   std::unique_ptr<MujocoSimBase> sim_;

   // for Log files
   std::fstream logFile_;
   std::string logFilePath_;

   // for upper body locked joints PD control
   int n_locked_joints_ = 0;
   VectorXd q_locked_joints_des_, dq_locked_joints_des_;
   PDController locked_joints_pd_controller_;
};

#endif // BASIC_STATE_MACHINE_HPP