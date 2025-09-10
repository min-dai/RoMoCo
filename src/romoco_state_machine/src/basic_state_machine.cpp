#include "romoco_state_machine/basic_state_machine.hpp"

#include <filesystem> //for create_directories


#include "romoco_control/torque_solver_tscqp.hpp"
// #include "romoco_control/torque_solver_tscqp_momentum.hpp"
#include "romoco_control/torque_solver_inv_dyn.hpp"
#include "romoco_control/torque_solver_vel_ik.hpp"
#include "romoco_control/torque_solver_pos_ik.hpp"

#include "romoco_output/walking_output_fp.hpp"

#include "romoco_output/standing_output.hpp"
#include "romoco_output/inair_output.hpp"

//enum classes
#include "romoco_screen_radio/radio_slider_map.hpp"
#include "romoco_types/biped_constants.hpp"
#include "romoco_types/biped_motor_commands.hpp"

BasicStateMachine::BasicStateMachine(const std::string &config_folder, const std::string &log_path, std::shared_ptr<RobotBasePinocchio> robot_ptr, std::unique_ptr<MujocoInterfaceBase> sim)
    : config_folder_(config_folder), log_path_(log_path), sim_(std::move(sim))
{
   Init(config_folder, log_path, robot_ptr);
}


//destructor
BasicStateMachine::~BasicStateMachine()
{
   Close();
}

void BasicStateMachine::Close()
{
   if (logFile_.is_open())
   {
      logFile_.close();
   }
}




void BasicStateMachine::Init(const std::string &config_folder, const std::string &log_path, std::shared_ptr<RobotBasePinocchio> robot_ptr)
{
   //  Initialize the robot config folder and log path
   std::string config_file = config_folder + "/interface_config.yaml";
   log_path_ = log_path; // log path = home + "/ROBOTLOG/" + robot_name ;

   // Check if the log directory exists, if not, create it
   if (!std::filesystem::exists(log_path_))
   {
      if (std::filesystem::create_directories(log_path_))
      {
         std::cout << "Log directory created: " << log_path_ << std::endl;
      }
      else
      {
         std::cerr << "Failed to create log directory: " << log_path_ << std::endl;
      }
   }

   YAMLParser yaml_parser(config_file);

   std::vector<std::string> locked_encoder_names = yaml_parser.get_string_vector("locked_encoder_names");
   n_locked_joints_ = locked_encoder_names.size();

   Eigen::VectorXd Kp_locked_joints, Kd_locked_joints;

   if (n_locked_joints_ > 0)
   {
      q_locked_joints_des_ = Eigen::VectorXd::Zero(n_locked_joints_);

      q_locked_joints_des_ = yaml_parser.get_VectorXd("qdes_locked_joints");
      dq_locked_joints_des_ = Eigen::VectorXd::Zero(n_locked_joints_);
      Kp_locked_joints = yaml_parser.get_VectorXd("Kp_locked_joints");
      Kd_locked_joints = yaml_parser.get_VectorXd("Kd_locked_joints");
      locked_joints_pd_controller_.Reconfigure(Kp_locked_joints, Kd_locked_joints);
   }
   locked_input_ = Eigen::VectorXd::Zero(n_locked_joints_);

   t_old_ = 0;

   qfull_ = Eigen::VectorXd::Zero(robot_ptr->nq() + n_locked_joints_);
   dqfull_ = Eigen::VectorXd::Zero(robot_ptr->nv() + n_locked_joints_);
   locomotion_input_ = Eigen::VectorXd::Zero(robot_ptr->nu()); // Adjust the size as needed
   motor_commands_ = BipedMotorCommands(robot_ptr->nu());
}

double BasicStateMachine::Update(const DesiredCommand &command,
                               std::shared_ptr<RobotBasePinocchio> robot_ptr,
                               std::shared_ptr<OutputBase> &output,
                               std::unique_ptr<TorqueSolverBase> &torque_solver,
                               std::function<Eigen::VectorXd(const Eigen::VectorXd&)> getLegModel,
                              std::function<Eigen::VectorXd(const Eigen::VectorXd&)> getUpper)
{
   // to be called after spinOnce inside ros::ok() loop
   //  Determine config file based on mode
   std::string config_file;
   if (command.mode == Mode::Standing)
   {
      config_file = config_folder_ + "/standing_config.yaml";
      logFilePath_ = log_path_ + "/logStand.bin";
   }
   else if (command.mode == Mode::InAir)
   {
      config_file = config_folder_ + "/inair_config.yaml";
      logFilePath_ = log_path_ + "/logInAir.bin";
      sim_->SimHoldPelvis();
   }
   else if (command.mode == Mode::Walking)
   {
      config_file = config_folder_ + "/walking_config.yaml";
      logFilePath_ = log_path_ + "/logWalk.bin";
   }
   else if (command.mode == Mode::Null)
   {
      //do nothing, just reset the state machine
   }
   else{
      std::cerr << "Invalid mode command: " << std::endl;
      std::cerr << "Did you forget to launch screen_radio?" << std::endl;
      //skip the rest of the update
      return sim_->sim_time();
   }

   if (cur_mode_ != command.mode)
   {
      bool canSwitch = true;
      if (cur_mode_ == Mode::Standing && command.mode == Mode::Walking)
      {
         canSwitch = output && output->is_ready_to_transit(); // Check OUTPUT_STAND
      }
      else if (cur_mode_ == Mode::Walking && command.mode == Mode::Standing)
      {
         canSwitch = output && output->is_ready_to_transit(); // Check OUTPUT_HLIP
      }

      if (canSwitch)
      {
         if (command.mode == Mode::Null)
         {
            // Reset to default state (stop controllers)
            if (output && torque_solver)
            {
               std::cout << "Resetting state machine to NULL mode." << std::endl;
               output.reset();        // Remove output
               torque_solver.reset(); // Remove Torque Solver
            }
            cur_mode_ = Mode::Null;
            if (logFile_.is_open())
            {
               logFile_.close();
            }
         }
         else
         {
            SelectControllers(command.mode, config_file, robot_ptr, output, torque_solver);
            cur_mode_ = command.mode;

            if (logFile_.is_open())
            {
               logFile_.close();
            }
            logFile_.open(logFilePath_, std::ios::out | std::ios::binary);
         }
      }
   }

   // Update the simulation, sensor info in simulation is also updated
   sim_->Step(locomotion_input_, locked_input_);

   if (!sim_->paused())
   {
      control_counter_++;
      if (control_counter_ >= control_counter_threshold_)
      {
         control_counter_ = 0;

         BipedProprioception loco_proprioception = sim_->ReadAndEstimate();
         sim_->ProcessMotorCommands(motor_commands_);
         sim_->SendPacket();
         sim_->GetAllJointStateFromSensorMujoco(qfull_, dqfull_);

         std::cout << "qfull_: " << qfull_.transpose() << std::endl;
         std::cout << "dqfull_: " << dqfull_.transpose() << std::endl;

         Eigen::VectorXd q_leg = getLegModel(qfull_);
         Eigen::VectorXd dq_leg = getLegModel(dqfull_);
         Eigen::VectorXd q_upper = getUpper(qfull_);
         Eigen::VectorXd dq_upper = getUpper(dqfull_);

         robot_ptr->UpdateAll(q_leg, dq_leg);


         if (output && torque_solver)
         {

            output->UpdateOutput(command, sim_->sim_time(), t_old_);
            t_old_ = sim_->sim_time();

            motor_commands_ = torque_solver->Solve();
            Eigen::VectorXd u_leg = motor_commands_.joint_torques;

            locomotion_input_ = u_leg;


            if (n_locked_joints_ > 0)
            {
               locked_input_ = locked_joints_pd_controller_.Compute(q_locked_joints_des_, dq_locked_joints_des_, q_upper, dq_upper);
            }
            
            // Logging
            Eigen::VectorXf LogData;

            std::vector<Eigen::VectorXd> log_vectors = {qfull_, dqfull_, locomotion_input_, locked_input_, output->ya_full(), output->dya_full(), output->yd_full(), output->dyd_full(), output->d2yd_full()};
            std::vector<Eigen::VectorXd> logOutput = output->CollectLog();
            log_vectors.insert(log_vectors.end(), logOutput.begin(), logOutput.end());
            LogData = CollectLog(sim_->sim_time(), log_vectors);

            if (logFile_.is_open())
            {
               logFile_.write(reinterpret_cast<char *>(LogData.data()), LogData.size() * sizeof(float));
            }
         }
      }
   }
   return sim_->sim_time();
}

Eigen::VectorXf BasicStateMachine::CollectLog(const double t, const std::vector<Eigen::VectorXd> &vectors)
{
   int logsize = 1; // Start with 1 for the time
   for (const auto &vec : vectors)
   {
      logsize += vec.size();
   }

   Eigen::VectorXf log(logsize);
   log(0) = static_cast<float>(t);
   int offset = 1; // Start after the constant
   for (const auto &vec : vectors)
   {
      log.segment(offset, vec.size()) = vec.cast<float>();
      offset += vec.size();
   }
   return log;
}

void BasicStateMachine::SelectControllers(
    Mode mode,
    const std::string &config_file,
    std::shared_ptr<RobotBasePinocchio> robot_ptr,
    std::shared_ptr<OutputBase> &output,
    std::unique_ptr<TorqueSolverBase> &torque_solver)
{
   // Load YAML configuration
   YAMLParser yaml_parser(config_file);
   std::string torque_solver_type = yaml_parser.get_string("controller");

   // Select output
   switch (mode)
   {
   case Mode::Standing:
      output = std::make_shared<StandingOutput>(config_file, robot_ptr);
      break;
   case Mode::InAir:
      output = std::make_shared<InAirOutput>(config_file, robot_ptr);
      break;
   case Mode::Walking:
      output = std::make_shared<WalkingOutputFp>(config_file, robot_ptr);
      // output = std::make_shared<WalkingOutputMultiDomainFP>(config_file, robot_ptr);
      break;
   default:
      std::cerr << "Invalid locomotion mode!" << std::endl;
      return;
   }

   // Select Torque Controller Based on YAML Configuration
   if (torque_solver_type == "qp")
   {
      torque_solver = std::make_unique<TorqueSolverTSCQP>(config_file, robot_ptr, output);
   }
   // else if (torque_solver_type == "qpm")
   // {
   //    torque_solver = std::make_unique<TorqueSolverTSCQPMomentum>(config_file, robot_ptr, output);
   // }
   else if (torque_solver_type == "id")
   {
      torque_solver = std::make_unique<TorqueSolverInvDyn>(config_file, robot_ptr, output);
   }
   else if (torque_solver_type == "velik")
   {
      torque_solver = std::make_unique<TorqueSolverVELIK>(config_file, robot_ptr, output);
   }
   else if (torque_solver_type == "posik")
   {
      torque_solver = std::make_unique<TorqueSolverPOSIK>(config_file, robot_ptr, output);
   }
   else
   {
      std::cerr << "Unknown torque_solver_type in YAML: " << torque_solver_type << std::endl;
      return;
   }
}
