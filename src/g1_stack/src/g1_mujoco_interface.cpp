#include "g1_mujoco_interface.hpp"

namespace romoco
{
G1MujocoInterface::G1MujocoInterface(const std::string &config_folder, const std::string &log_path)
{
   Init(config_folder, log_path);
}
G1MujocoInterface::G1MujocoInterface(const std::string &config_folder, const std::string &log_path, std::unique_ptr<romoco::robot::RobotBasePinocchio> robot)
    : robot_(std::move(robot)),
      contact_kf_(std::make_unique<ContactKf>(config_folder, 6))
{
   Init(config_folder, log_path);
}
G1MujocoInterface::~G1MujocoInterface()
{
   std::cout << "G1 Mujoco interface destructor called." << std::endl;
}

void G1MujocoInterface::Init(const std::string &config_folder, const std::string &log_path)
{
   InitLogFile(log_path);

   InitDofAndIndicesFromConfigFile(config_folder);
   InitMotorCommands();

   std::string interface_config_file = config_folder + "/interface_config.yaml";
   ReconfigurePdMotorCommands(interface_config_file);
   InitProprioception();

   // Init Mujoco
   std::string config_file = config_folder + "/mujoco_config.yaml";
   YAMLParser yaml_parser(config_file);

   std::string model_name = yaml_parser.get_string("mujoco_settings/model_name");
   videoSetting.InitVideoSetting(config_file);

   const std::string model_path = config_folder + "/../model_files/" + model_name;

   mujoco_.Init(model_path.c_str(), videoSetting.video_width, videoSetting.video_height);

   pin_pelvis_ = yaml_parser.get_bool("mujoco_settings/pin_pelvis");
   if (pin_pelvis_)
   {
      SimHoldPelvis();
   }

   gyro_mj_ids = mujoco_.GetSensorIdsByName(gyro_name);
   accelerometer_mj_ids = mujoco_.GetSensorIdsByName(accelerometer_name);
   framequat_mj_ids = mujoco_.GetSensorIdsByName(framequat_name);

   all_encoder_mj_ids_pinocchio_order = mujoco_.GetJointIdsByName(all_encoder_names_pinocchio_order_);

   locomotion_actuator_mj_ids_ = mujoco_.GetActuatorIdsByName(loco_encoder_names_);
   locked_actuator_mj_ids_ = mujoco_.GetActuatorIdsByName(pd_encoder_names_);

   std::vector<std::string> modify_joint_names = {"left_knee_joint", "right_knee_joint", "left_hip_pitch_joint", "right_hip_pitch_joint", "left_ankle_pitch_joint", "right_ankle_pitch_joint"};
   std::vector<int> modify_joint_mj_ids = mujoco_.GetJointIdsByName(modify_joint_names);
   Eigen::VectorXd modify_joint_pos(6);
   modify_joint_pos << 0.4, 0.4, -0.2, -0.2, -.2, -.2;

   mujoco_.set_1dof_joint_qpos(modify_joint_pos, modify_joint_mj_ids);

   if (locked_actuator_mj_ids_.size() > 0)
   {
      std::vector<int> upper_joint_mj_ids = mujoco_.GetJointIdsByName(pd_encoder_names_);
      mujoco_.set_1dof_joint_qpos(pd_motor_commands_.joint_positions, upper_joint_mj_ids);
   }

   // double yaw = 3.14159265 / 2. * 3;
   // mujoco_.set_base_quaternion(Eigen::Vector4d(cos(yaw / 2.), 0., 0., sin(yaw / 2.)));
   mujoco_.set_zero_qvel();
   mujoco_.set_zero_qacc();
   mujoco_.Sim1StepForward();
   if (videoSetting.record_video)
   {
      mujoco_.StartVideoRecording(videoSetting.video_name.c_str(), videoSetting.video_fps);
      std::cout << "Recording video..." << std::endl;
   }
   std::cout << "G1 Mujoco simulation initialized successfully." << std::endl;

   // Initialize torques

   torque_loco_ = Eigen::VectorXd::Zero(locomotion_actuator_mj_ids_.size());
   torque_pd_ = Eigen::VectorXd::Zero(locked_actuator_mj_ids_.size());

   // Initial step to set up sensor_
   if (!Step(torque_loco_, torque_pd_))
   {
      throw std::runtime_error("Initial simulation step failed");
   }
   if (robot_ != nullptr)
   {
      robot_->ReconfigureContactClassifier(config_folder);
      sensor_hw_.ResizeAll(total_motor_dof_);
      std::cout << "Initial Sensor Data :" << sensor_hw_ << std::endl;
   }
   else
   {
      std::cout << "Initial Sensor Data :" << sensor_ << std::endl;
   }
}

bool G1MujocoInterface::Step(const Eigen::VectorXd &leg_control_input, const Eigen::VectorXd &upper_control_input)
{
   if (!mujoco_.IsWindowOpen())
   {
      std::cout << "Window closed, exiting simulation." << std::endl;
      mujoco_.Close();
      return 0;
   }

   if (!mujoco_.paused())
   {

      mujoco_.UpdateControlInput(leg_control_input, locomotion_actuator_mj_ids_);
      if (locked_actuator_mj_ids_.size() > 0)
      {
         mujoco_.UpdateControlInput(upper_control_input, locked_actuator_mj_ids_);
      }
      mujoco_.Sim1StepForward();

      Eigen::VectorXf LogData;

      std::vector<Eigen::VectorXd> log_vectors = {est_lin_vel_, true_lin_vel_};
      LogData = CollectLog(sim_time(), log_vectors);
      if (logFile_.is_open())
      {
         logFile_.write(reinterpret_cast<char *>(LogData.data()), LogData.size() * sizeof(float));
      }
   }

   if (robot_ != nullptr)
   {
      UpdateHardwareSensorData();
   }
   else
   {
      UpdateMujocoTrueSensorData();
   }

   HandleRendering();

   return 1;
}

void G1MujocoInterface::UpdateMujocoTrueSensorData()
{
   sensor_.encoders_pos_pinocchio_order = mujoco_.GetJointPositionsByIds(all_encoder_mj_ids_pinocchio_order);
   sensor_.encoders_vel_pinocchio_order = mujoco_.GetJointVelocitiesByIds(all_encoder_mj_ids_pinocchio_order);

   // Update base position and velocity
   const auto *qpos = mujoco_.qpos();
   const auto *qvel = mujoco_.qvel();

   sensor_.base_lin_pos << qpos[0], qpos[1], qpos[2];
   sensor_.base_lin_vel << qvel[0], qvel[1], qvel[2];

   // Update base orientation (quaternion: w, x, y, z)
   sensor_.base_ang_quat.w() = qpos[3];
   sensor_.base_ang_quat.x() = qpos[4];
   sensor_.base_ang_quat.y() = qpos[5];
   sensor_.base_ang_quat.z() = qpos[6];

   sensor_.base_ang_vel << qvel[3], qvel[4], qvel[5];
}

void G1MujocoInterface::UpdateHardwareSensorData()
{

   sensor_hw_.encoders_pos_pinocchio_order = mujoco_.GetJointPositionsByIds(all_encoder_mj_ids_pinocchio_order);
   sensor_hw_.encoders_vel_pinocchio_order = mujoco_.GetJointVelocitiesByIds(all_encoder_mj_ids_pinocchio_order);
   Eigen::VectorXd quat = mujoco_.GetSensorDataByIds(framequat_mj_ids);
   sensor_hw_.base_ang_quat.w() = quat[0];
   sensor_hw_.base_ang_quat.x() = quat[1];
   sensor_hw_.base_ang_quat.y() = quat[2];
   sensor_hw_.base_ang_quat.z() = quat[3];
   sensor_hw_.base_ang_vel = mujoco_.GetSensorDataByIds(gyro_mj_ids);
   sensor_hw_.base_lin_acc = mujoco_.GetSensorDataByIds(accelerometer_mj_ids);
   // todo: remove gravity
   //  sensor_hw_.base_lin_acc(2) -= 9.81;
   const auto *qvel = mujoco_.qvel();
   true_lin_vel_ << qvel[0], qvel[1], qvel[2];
}

void G1MujocoInterface::Estimate()
{

   BipedProprioception proprioception = GetBipedProprioceptionFromRawSensorDataHardware(sensor_hw_);
   std::cout << "sensor_hw" << std::endl;
   std::cout << sensor_hw_ << std::endl;
   std::cout << "proprioception" << std::endl;
   std::cout << proprioception << std::endl;
   robot_->UpdateKinematics(proprioception.q(loco_proprio_indices_), proprioception.qdot(loco_proprio_indices_));

   BipedEstimationKinematicsInput biped_estimation_kinematics_input = robot_->ComputeEstimationKinematicsInput();
   est_lin_vel_ = contact_kf_->Update(sim_time(), sensor_hw_.base_lin_acc, sensor_hw_.base_ang_quat, biped_estimation_kinematics_input);

   proprioception.qdot.head(3) << est_lin_vel_;
   full_proprioception_ = proprioception;
}

BipedProprioception G1MujocoInterface::ReadAndEstimate()
{
   if (robot_ != nullptr)
   {
      Estimate();
      loco_proprioception_ = full_proprioception_.GetIndex(loco_proprio_indices_);
      pd_proprioception_ = full_proprioception_.GetIndex(pd_proprio_indices_);
      return loco_proprioception_;
   }
   else
   {
      return MujocoInterfaceBase::ReadAndEstimate();
   }
}
void G1MujocoInterface::SendPacket()
{
   MujocoInterfaceBase::SendPacket();
   std::cout << "Loco Torque Command: " << torque_loco_.transpose() << std::endl;
   if (robot_ != nullptr)
   {
      robot_->set_computed_torque(torque_loco_);
   }
}

void G1MujocoInterface::HandleRendering()
{
   render_loop_counter_++;
   if (render_loop_counter_ % render_loop_counter_threshold_ == 0)
   {
      mujoco_.Render();
      render_loop_counter_ = 0;

      if (videoSetting.record_video && !mujoco_.paused())
      {
         mujoco_.RecordVideoFrame();
      }
   }
}

} // namespace romoco