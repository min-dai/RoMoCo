#include "romoco_state_machine/basic_controller_state_machine.hpp"

#include <filesystem> //for create_directories


#include "romoco_control/torque_solver_tscqp.hpp"
// #include "romoco_control/torque_solver_tscqp_momentum.hpp"
#include "romoco_control/torque_solver_inv_dyn.hpp"
#include "romoco_control/torque_solver_vel_ik.hpp"
#include "romoco_control/torque_solver_pos_ik.hpp"

#include "romoco_output/walking_output_fp.hpp"
#include "romoco_output/walking_output_multidomain_fp.hpp"
#include "romoco_output/standing_output.hpp"
#include "romoco_output/inair_output.hpp"

//enum classes
#include "romoco_types/biped_constants.hpp"


BasicControllerStateMachine::BasicControllerStateMachine(const std::string &config_folder, const std::string &log_path, std::shared_ptr<RobotBasePinocchio> robot_ptr)
    : config_folder_(config_folder), log_path_(log_path)
{
   Init(config_folder, log_path, robot_ptr);
}


//destructor
BasicControllerStateMachine::~BasicControllerStateMachine()
{
   Close();
}

void BasicControllerStateMachine::Close()
{
   if (logFile_.is_open())
   {
      logFile_.close();
   }
}




void BasicControllerStateMachine::Init(const std::string &config_folder, const std::string &log_path, std::shared_ptr<RobotBasePinocchio> robot_ptr)
{
   //  Initialize the robot config folder and log path
   std::string mujoco_config_file = config_folder + "/mujoco_config.yaml";
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

   YAMLParser yaml_parser(mujoco_config_file);


   timer_.Reset();


   locomotion_input_ = VectorXd::Zero(robot_ptr->nu()); // Adjust the size as needed
}

BipedMotorCommands BasicControllerStateMachine::UpdateControl(const DesiredCommand &command,
                               std::shared_ptr<RobotBasePinocchio> robot_ptr,
                               std::shared_ptr<OutputBase> &output,
                               std::unique_ptr<TorqueSolverBase> &torque_solver,
                               Eigen::VectorXd& q_loco, Eigen::VectorXd& dq_loco)
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
      return motor_commands_;
   }

   if (cur_mode_ != command.mode)
   {
      bool canSwitch = true;
      if (cur_mode_ == Mode::Standing && command.mode == Mode::Walking)
      {
         canSwitch = output && output->isReadyToTransition(); // Check OUTPUT_STAND
      }
      else if (cur_mode_ == Mode::Walking && command.mode == Mode::Standing)
      {
         canSwitch = output && output->isReadyToTransition(); // Check OUTPUT_HLIP
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
               motor_commands_.ZeroAll(); // Zero out motor commands
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
            switch (cur_mode_)
            {
            case Mode::Standing:
               std::cout << "Switched to STANDING mode." << std::endl;
               break;
            case Mode::InAir:
               std::cout << "Switched to IN-AIR mode." << std::endl;
               break;
            case Mode::Walking:
               std::cout << "Switched to WALKING mode." << std::endl;
               break;
            default:
               break;
            }

            if (logFile_.is_open())
            {
               logFile_.close();
            }
            logFile_.open(logFilePath_, std::ios::out | std::ios::binary);
         }
      }
   }

         timer_.Tick();

         if (output && torque_solver)
         {
            robot_ptr->UpdateAll(q_loco, dq_loco);
            output->UpdateOutput(command, timer_.ElapsedSinceStart(), timer_.t_old());


            motor_commands_ = torque_solver->Solve();
            locomotion_input_ = motor_commands_.joint_torques;
            

            // Logging
            Eigen::VectorXf LogData;

            std::vector<Eigen::VectorXd> log_vectors = {q_loco, dq_loco, locomotion_input_, output->ya, output->dya, output->yd, output->dyd, output->d2yd};
            std::vector<Eigen::VectorXd> logOutput = output->CollectLog();
            log_vectors.insert(log_vectors.end(), logOutput.begin(), logOutput.end());
            LogData = CollectLog(timer_.ElapsedSinceStart(), log_vectors);

            if (logFile_.is_open())
            {
               logFile_.write(reinterpret_cast<char *>(LogData.data()), LogData.size() * sizeof(float));
            }
         }
      
   
   return motor_commands_;
}

Eigen::VectorXf BasicControllerStateMachine::CollectLog(const double t, const std::vector<VectorXd> &vectors)
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

void BasicControllerStateMachine::SelectControllers(
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
