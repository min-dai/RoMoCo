#include "g1_mujoco_sim.hpp"
#include "biped_utils/yaml_parser.hpp"

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
   std::string config_file = config_folder + "/mujoco_config.yaml";
   YAMLParser yaml_parser(config_file);

   std::string model_name = yaml_parser.get_string("mujoco_settings/model_name");

   videoSetting.InitVideoSetting(config_file);

   char modelfile[500] = "";
   strcat(modelfile, config_folder.c_str());
   strcat(modelfile, "/../model_files/");
   strcat(modelfile, model_name.c_str());

   mujoco_.Init(modelfile, videoSetting.video_width, videoSetting.video_height);

   std::string interface_config_file = config_folder + "/interface_config.yaml";
   yaml_parser.Init(interface_config_file);
   std::vector<std::string> locked_encoder_names = yaml_parser.get_string_vector("locked_encoder_names");
   std::vector<std::string> pinocchio_encoder_names = yaml_parser.get_string_vector("pinocchio_encoder_names");
   //verify they add up to all
   if (locked_encoder_names.size() + pinocchio_encoder_names.size() != all_encoder_names_pinocchio_order.size())
   {
      throw std::runtime_error("Interface Error: Locked and locomotion encoder names do not add up to all encoders");
   }
   total_motor_dof_ = all_encoder_names_pinocchio_order.size();
   loco_motor_indices_ = GetJointIndicesFromSubset(all_encoder_names_pinocchio_order, pinocchio_encoder_names);
   pd_motor_indices_ = GetJointIndicesFromSubset(all_encoder_names_pinocchio_order, locked_encoder_names);

   total_proprio_dof_ = 6 + total_motor_dof_;

   // proprio indices = base + shifted motors
   loco_proprio_indices_ = {0, 1, 2, 3, 4, 5};
   loco_proprio_indices_.reserve(6 + loco_motor_indices_.size());

   // append shifted motor indices
   std::transform(loco_motor_indices_.begin(),
                  loco_motor_indices_.end(),
                  std::back_inserter(loco_proprio_indices_),
                  [](int idx)
                  { return idx + 6; });

   pd_proprio_indices_ = pd_motor_indices_;

   for (int &x : pd_proprio_indices_)
   {
      x += 6;
   }

   InitMotorCommands();
   InitProprioception();

   gyro_ids = mujoco_.GetSensorIdsByName(gyro_name);
   accelerometer_ids = mujoco_.GetSensorIdsByName(accelerometer_name);
   framequat_ids = mujoco_.GetSensorIdsByName(framequat_name);

   all_encoder_ids_pinocchio_order = mujoco_.GetJointIdsByName(all_encoder_names_pinocchio_order);

   locomotion_actuator_ids_ = mujoco_.GetActuatorIdsByName(pinocchio_encoder_names);
   locked_actuator_ids_ = mujoco_.GetActuatorIdsByName(locked_encoder_names);

   std::vector<std::string> modify_joint_names = {"left_knee_joint", "right_knee_joint", "left_hip_pitch_joint", "right_hip_pitch_joint", "left_ankle_pitch_joint", "right_ankle_pitch_joint"};
   std::vector<int> modify_joint_ids = mujoco_.GetJointIdsByName(modify_joint_names);
   Eigen::VectorXd modify_joint_pos(6);
   modify_joint_pos << 0.4, 0.4, -0.2, -0.2, -.2, -.2;
   mujoco_.set_1dof_joint_qpos(modify_joint_pos, modify_joint_ids);

   if (locked_actuator_ids_.size() > 0)
   {
      Eigen::VectorXd upper_joint_pos = yaml_parser.get_VectorXd("qdes_locked_joints");
      std::vector<int> upper_joint_ids = mujoco_.GetJointIdsByName(locked_encoder_names);
      mujoco_.set_1dof_joint_qpos(upper_joint_pos, upper_joint_ids);
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
      mujoco_.UpdateControlInput(leg_control_input, locomotion_actuator_ids_);
      if (locked_actuator_ids_.size() > 0)
      {
         mujoco_.UpdateControlInput(upper_control_input, locked_actuator_ids_);
      }
      mujoco_.Sim1StepForward();
   }

   sensor_.encoders_pos_pinocchio_order = mujoco_.GetJointPositionsByIds(all_encoder_ids_pinocchio_order);
   sensor_.encoders_vel_pinocchio_order = mujoco_.GetJointVelocitiesByIds(all_encoder_ids_pinocchio_order);

   sensor_.base_lin_pos << mujoco_.qpos()[0], mujoco_.qpos()[1], mujoco_.qpos()[2];
   sensor_.base_lin_vel << mujoco_.qvel()[0], mujoco_.qvel()[1], mujoco_.qvel()[2];
   sensor_.base_ang_quat.w() = mujoco_.qpos()[3];
   sensor_.base_ang_quat.x() = mujoco_.qpos()[4];
   sensor_.base_ang_quat.y() = mujoco_.qpos()[5];
   sensor_.base_ang_quat.z() = mujoco_.qpos()[6];
   sensor_.base_ang_vel << mujoco_.qvel()[3], mujoco_.qvel()[4], mujoco_.qvel()[5];

   


   mujoco_.Render();

   if (videoSetting.record_video && !mujoco_.paused())
   {
      mujoco_.RecordVideoFrame();
   }

   return 1;
}

