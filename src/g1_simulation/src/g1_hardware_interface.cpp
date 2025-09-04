


#include "g1_hardware_interface.hpp"
#include "biped_core/prep_proprioception.hpp"
#include <iostream>
#include <thread>
#include <chrono>

G1HardwareInterface::G1HardwareInterface(const std::string &network_interface, const std::string &config_folder, const std::string &log_path, std::unique_ptr<RobotBasePinocchio> robot)
    : mode_pr_(G1Mode::PR),  // Default to PR mode
      mode_machine_(0),
      robot_(std::move(robot)),
      contact_kf_(std::make_unique<ContactKf>(config_folder, 6))

{
    Init(config_folder, log_path);

    // Initialize DDS
    ChannelFactory::Instance()->Init(0, network_interface);
    
    // Initialize motion switcher
    motion_switcher_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
    motion_switcher_->SetTimeout(5.0f);
    motion_switcher_->Init();
    // Release any existing motion control
    std::string form, name;
    while (motion_switcher_->CheckMode(form, name), !name.empty()) {
        if (motion_switcher_->ReleaseMode()) {
            std::cerr << "Failed to switch to Release Mode\n";
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    
    // create publisher
    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();
    // create subscriber
    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(std::bind(&G1HardwareInterface::LowStateHandler, this, std::placeholders::_1), 1);
    torso_imu_subscriber_.reset(new ChannelSubscriber<IMUState_>(HG_IMU_TORSO));
    torso_imu_subscriber_->InitChannel(std::bind(&G1HardwareInterface::TorsoImuHandler, this, std::placeholders::_1), 1);
    // create threads
    command_writer_ptr_ = CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, 2000, &G1HardwareInterface::LowCommandWriter, this);
    control_thread_ptr_ = CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, 2000, &G1HardwareInterface::SendPacket, this);

    simple_timer_.Reset();



    if (!robot_){
        throw std::runtime_error("G1HardwareInterface: Robot model is not provided.");
    }else{
        robot_->ReconfigureContactClassifier(0.0005);
    }

    std::cout << "G1 Hardware Interface initialized on " << network_interface << std::endl;
}



void G1HardwareInterface::Init(const std::string &config_folder, const std::string &log_path)
{
    InitLogFile(log_path, false);
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
    
    
    // Initialize base class structures
    InitMotorCommands();
    std::string interface_config_file = config_folder + "/interface_config.yaml";
   ReconfigurePdMotorCommands(interface_config_file);
   InitProprioception();
    
    sensor_hw_.encoders_pos_pinocchio_order.resize(G1_NUM_MOTOR);
    sensor_hw_.encoders_vel_pinocchio_order.resize(G1_NUM_MOTOR);
}



void G1HardwareInterface::LowStateHandler(const void* message)
{
    LowState_ low_state = *(const LowState_*)message;

    // Size validation
    if (low_state.motor_state().size() < G1_NUM_MOTOR) {
        std::cerr << "ERROR: motor_state size " << low_state.motor_state().size() 
                  << " < expected " << G1_NUM_MOTOR << std::endl;
        return;
    }
    
    // Verify CRC
    if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1)) {
      std::cout << "[ERROR] CRC Error" << std::endl;
      return;
    }
    
    // Extract motor state
    MotorState ms_tmp;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        ms_tmp.q.at(i) = low_state.motor_state()[i].q();
        ms_tmp.dq.at(i) = low_state.motor_state()[i].dq();
      if (low_state.motor_state()[i].motorstate() && i <= 11)
        std::cout << "[ERROR] motor " << i << " with code " << low_state.motor_state()[i].motorstate() << "\n";
    }
    motor_state_buffer_.SetData(ms_tmp);
    
    // Extract pelvis IMU state
    ImuState pelvis_imu;
    pelvis_imu.quat = low_state.imu_state().quaternion();
    pelvis_imu.rpy = low_state.imu_state().rpy();
    pelvis_imu.omega = low_state.imu_state().gyroscope();
    pelvis_imu.acc = low_state.imu_state().accelerometer();
    pelvis_imu_buffer_.SetData(pelvis_imu);

    // update gamepad
    // Add bounds checking before memcpy
    if (low_state.wireless_remote().size() >= 40 && sizeof(rx_.buff) >= 40)
    {
        memcpy(rx_.buff, &low_state.wireless_remote()[0], 40);
    }
    else
    {
        std::cerr << "Buffer size mismatch!" << std::endl;
    }
    gamepad_.update(rx_.RF_RX);

    // update mode machine
    if (mode_machine_ != low_state.mode_machine()) {
      if (mode_machine_ == 0) std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;
      mode_machine_ = low_state.mode_machine();
    }
}

void G1HardwareInterface::TorsoImuHandler(const void* message)
{
    IMUState_ imu_msg = *(const IMUState_*)message;
    
    ImuState torso_imu;
    torso_imu.quat = imu_msg.quaternion();
    torso_imu.rpy = imu_msg.rpy();
    torso_imu.omega = imu_msg.gyroscope();
    torso_imu.acc = imu_msg.accelerometer();
    torso_imu_buffer_.SetData(torso_imu);
}

void G1HardwareInterface::ReadAndEstimate()
{
    // Get latest state from buffers
    const std::shared_ptr<const MotorState> motor_state = motor_state_buffer_.GetData();
    const std::shared_ptr<const ImuState> pelvis_imu = pelvis_imu_buffer_.GetData();

    if (!motor_state || !pelvis_imu) {
        return;  // No data available yet
    }

    ConvertG1StateToProprioception(*motor_state, *pelvis_imu);

    Eigen::VectorXf LogData;

    std::vector<Eigen::VectorXd> log_vectors = {est_lin_vel_};
    LogData = CollectLog(simple_timer_.ElapsedSinceStart(), log_vectors);
    if (logFile_.is_open())
    {
        logFile_.write(reinterpret_cast<char *>(LogData.data()), LogData.size() * sizeof(float));
    }
}

void G1HardwareInterface::ConvertG1StateToProprioception(
    const MotorState& motor_state,
    const ImuState& pelvis_imu)
{
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        sensor_hw_.encoders_pos_pinocchio_order(i) = static_cast<double>(motor_state.q[i]);
        sensor_hw_.encoders_vel_pinocchio_order(i) = static_cast<double>(motor_state.dq[i]);
    }

    sensor_hw_.base_ang_quat.w() = static_cast<double>(pelvis_imu.quat[0]);
    sensor_hw_.base_ang_quat.x() = static_cast<double>(pelvis_imu.quat[1]);
    sensor_hw_.base_ang_quat.y() = static_cast<double>(pelvis_imu.quat[2]);
    sensor_hw_.base_ang_quat.z() = static_cast<double>(pelvis_imu.quat[3]);

    sensor_hw_.base_ang_vel << static_cast<double>(pelvis_imu.omega[0]),
                               static_cast<double>(pelvis_imu.omega[1]),
                               static_cast<double>(pelvis_imu.omega[2]);

    sensor_hw_.base_lin_acc << static_cast<double>(pelvis_imu.acc[0]),
                               static_cast<double>(pelvis_imu.acc[1]),
                               static_cast<double>(pelvis_imu.acc[2]);


    BipedProprioception proprioception = GetBipedProprioceptionFromRawSensorDataHardware(sensor_hw_);

    std::cout << "proprioception q: " << proprioception.q(loco_proprio_indices_).transpose() << std::endl;
    std::cout << "proprioception qdot: " << proprioception.qdot(loco_proprio_indices_).transpose() << std::endl;

    robot_->UpdateKinematics(proprioception.q(loco_proprio_indices_), proprioception.qdot(loco_proprio_indices_));

    simple_timer_.Tick();
   BipedEstimationKinematicsInput biped_estimation_kinematics_input = robot_->ComputeEstimationKinematicsInput();
   est_lin_vel_ = contact_kf_->Update(simple_timer_.ElapsedSinceStart(), sensor_hw_.base_lin_acc, sensor_hw_.base_ang_quat, biped_estimation_kinematics_input);

   proprioception.qdot.head(3) << est_lin_vel_;
   full_proprioception_ = proprioception;
    
    // Update subsets
    loco_proprioception_ = full_proprioception_.GetIndex(loco_proprio_indices_);
    pd_proprioception_ = full_proprioception_.GetIndex(pd_proprio_indices_);
}

void G1HardwareInterface::SendPacket()
{
    // Convert final motor commands to G1 format
    final_motor_commands_.ZeroAll();
    loco_motor_commands_.ZeroAll();
    pd_motor_commands_.ZeroAll();
    ConvertMotorCommandsToG1(final_motor_commands_);

    torque_loco_ = loco_motor_commands_.SolveFullTorque(loco_proprioception_.q(motored_loco_proprio_indices_), loco_proprioception_.qdot(motored_loco_proprio_indices_));
   
    robot_->SetComputedTorque(torque_loco_);

}

void G1HardwareInterface::ConvertMotorCommandsToG1(const BipedMotorCommands& commands)
{
    MotorCommand mc_tmp;
    
    // Convert from proprioception space to G1 space
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        mc_tmp.q_target[i] = commands.joint_positions(i);
        mc_tmp.dq_target[i] = commands.joint_velocities(i);
        mc_tmp.kp[i] = commands.joint_kp(i);
        mc_tmp.kd[i] = commands.joint_kd(i);
        mc_tmp.tau_ff[i] = commands.joint_torques_ff(i);
    }
    
    // Store in buffer for command writer thread
    motor_command_buffer_.SetData(mc_tmp);
}

void G1HardwareInterface::LowCommandWriter()
{
    LowCmd_ dds_low_command;
    dds_low_command.mode_pr() = static_cast<uint8_t>(mode_pr_);
    dds_low_command.mode_machine() = mode_machine_;

    const std::shared_ptr<const MotorCommand> mc = motor_command_buffer_.GetData();
    if (mc) {
      for (size_t i = 0; i < G1_NUM_MOTOR; i++) {
        dds_low_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable
        dds_low_command.motor_cmd().at(i).tau() = mc->tau_ff.at(i);
        dds_low_command.motor_cmd().at(i).q() = mc->q_target.at(i);
        dds_low_command.motor_cmd().at(i).dq() = mc->dq_target.at(i);
        dds_low_command.motor_cmd().at(i).kp() = mc->kp.at(i);
        dds_low_command.motor_cmd().at(i).kd() = mc->kd.at(i);
      }

      dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
      lowcmd_publisher_->Write(dds_low_command);
    }
}

uint32_t G1HardwareInterface::Crc32Core(uint32_t* ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    
    for (uint32_t i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else {
                CRC32 <<= 1;
            }
            if (data & xbit) {
                CRC32 ^= dwPolynomial;
            }
            xbit >>= 1;
        }
    }
    return CRC32;
}



