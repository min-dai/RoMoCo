#include "g1_mujoco_sim.hpp"

G1MujocoSim::G1MujocoSim(const std::string &config_file)
{
   Init(config_file);
}
G1MujocoSim::~G1MujocoSim()
{
   std::cout << "G1 Mujoco simulation destructor called." << std::endl;
}




void G1MujocoSim::Init(const std::string &config_folder)
{
      all_encoder_names_pinocchio_order_ = {
       "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
       "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
       "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
       "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
       "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
       "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
       "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
       "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
       "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"};


   InitDofAndIndicesFromConfigFile(config_folder);

      
   InitMotorCommands();
   std::cout << "Init Motor Commands" << std::endl;
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

   torque_loco_ = Eigen::VectorXd::Zero(locomotion_actuator_mj_ids_.size());
   torque_pd_ = Eigen::VectorXd::Zero(locked_actuator_mj_ids_.size());

   // Initial step to set up sensor_
   if (!Step(torque_loco_, torque_pd_))
   {
      throw std::runtime_error("Initial simulation step failed");
   }
   std::cout << "sensor" << sensor_ << std::endl;
}

bool G1MujocoSim::Step(const Eigen::VectorXd &leg_control_input, const Eigen::VectorXd &upper_control_input)
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
   }

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

   return 1;
}
