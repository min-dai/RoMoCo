

#include "g1_hardware_interface.hpp"
#include "romoco_core/prep_proprioception.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

G1HardwareInterface::G1HardwareInterface(const std::string &network_interface, const std::string &config_folder, const std::string &log_path, std::unique_ptr<RobotBasePinocchio> robot)
    : mode_pr_(G1Mode::PR), // Default to PR mode
      mode_machine_(0),
      robot_(std::move(robot)),
      contact_kf_(std::make_unique<ContactKf>(config_folder, 6)),
      InterfaceBase(false)
{
    Init(config_folder, log_path);
    InitDDS(network_interface);
    // 1 if active, 0 if inactive
    g1_motor_mode = {1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,0,0,1,1,1,1,1,0,0}; 
    
    


    //update t0
    simple_timer_.Reset();

    if (!robot_)
    {
        throw std::runtime_error("G1HardwareInterface: Robot model is not provided.");
    }
    else
    {
        robot_->ReconfigureContactClassifier(config_folder);
    }

    std::cout << "G1 Hardware Interface initialized on " << network_interface << std::endl;
}
void G1HardwareInterface::GetInitialProprioception()
{
    ReadAndEstimate();
    pd_qm0_ = pd_proprioception_.q;
    loco_qm0_ = loco_proprioception_.q(loco_proprio_motor_indices_);
    initial_proprioception_received_ = true;
    simple_timer_.Reset();
    std::cout << "Initial pd_qm0_: " << pd_qm0_.transpose() << std::endl;
    std::cout << "Initial loco_qm0_: " << loco_qm0_.transpose() << std::endl;
}
void G1HardwareInterface::Init(const std::string &config_folder, const std::string &log_path)
{
    InitLogFile(log_path);

    InitDofAndIndicesFromConfigFile(config_folder);

    // Initialize base class structures
    InitMotorCommands();



    interface_config_file_ = config_folder + "/interface_config.yaml";
    ReconfigurePdMotorCommands(interface_config_file_);
    InitProprioception();

    sensor_hw_.ResizeAll(total_motor_dof_);

    YAMLParser yaml_parser(interface_config_file_);
    default_pd_q_ = yaml_parser.get_VectorXd("qdes_locked_joints");
    default_loco_q_ = yaml_parser.get_VectorXd("default_loco_q");

    q_lower_bound_ = yaml_parser.get_VectorXd("safe_loco_q_bounds/lower");
    q_upper_bound_ = yaml_parser.get_VectorXd("safe_loco_q_bounds/upper");


}

void G1HardwareInterface::DampedInitializationControl()
{
    if (!initial_proprioception_received_)
    {
        GetInitialProprioception();
    }

    double t = simple_timer_.ElapsedSinceStart();
    double Tinit = 5.0; // Duration of initialization phase in seconds
    Eigen::VectorXd pd_qm_t, loco_qm_t;
    if (t < Tinit)
    {
        ReadAndEstimate();
        pd_qm_t = t * (default_pd_q_ - pd_qm0_) / Tinit + pd_qm0_;
        loco_qm_t = t * (default_loco_q_ - loco_qm0_) / Tinit + loco_qm0_;

        final_motor_commands_.joint_positions(loco_motor_command_indices_) << loco_qm_t;
        final_motor_commands_.joint_positions(pd_motor_command_indices_) << pd_qm_t;
        final_motor_commands_.joint_velocities.setZero();
        final_motor_commands_.joint_torques_ff.setZero();
        final_motor_commands_.joint_kp.setConstant(40.0);
        final_motor_commands_.joint_kd.setConstant(1.0);

        // Convert final motor commands to G1 format
        ConvertMotorCommandsToG1(final_motor_commands_);

        // Send commands to robot
        LowCommandWriter();
    }
    else
    {
        ready_for_control_ = true;
        std::cout << "G1 Hardware Interface is ready for control." << std::endl;
        // change pd back to normal
        ReconfigurePdMotorCommands(interface_config_file_);
        loco_motor_commands_.joint_positions = default_loco_q_;
        loco_motor_commands_.joint_velocities.setZero();
        loco_motor_commands_.joint_torques_ff.setZero();
        loco_motor_commands_.joint_kp.setConstant(40.0);
        loco_motor_commands_.joint_kd.setConstant(1.0);
    }
}

void G1HardwareInterface::InitDDS(const std::string &network_interface)
{
    // Initialize DDS
    ChannelFactory::Instance()->Init(0, network_interface);

    // Initialize motion switcher
    motion_switcher_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
    motion_switcher_->SetTimeout(5.0f);
    motion_switcher_->Init();
    // Release any existing motion control
    std::string form, name;
    while (motion_switcher_->CheckMode(form, name), !name.empty())
    {
        if (motion_switcher_->ReleaseMode())
        {
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
    // torso_imu_subscriber_.reset(new ChannelSubscriber<IMUState_>(HG_IMU_TORSO));
    // torso_imu_subscriber_->InitChannel(std::bind(&G1HardwareInterface::TorsoImuHandler, this, std::placeholders::_1), 1);
    
}

void G1HardwareInterface::LowStateHandler(const void *message)
{
    LowState_ low_state = *(const LowState_ *)message;

    // Size validation
    if (low_state.motor_state().size() < total_motor_dof_)
    {
        std::cerr << "ERROR: motor_state size " << low_state.motor_state().size()
                  << " < expected " << total_motor_dof_ << std::endl;
        return;
    }

    // Verify CRC
    if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1))
    {
        std::cout << "[ERROR] CRC Error" << std::endl;
        return;
    }

    // Extract motor state
    MotorState ms_tmp;
    for (int i = 0; i < total_motor_dof_; ++i)
    {
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
    if (gamepad_.X.pressed)
    {
        //exit program when X button is pressed
        // this is kill swtich
        std::cout << "X button pressed, exiting program." << std::endl;
        std::exit(0);
    }

    // update mode machine
    if (mode_machine_ != low_state.mode_machine())
    {
        if (mode_machine_ == 0)
            std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;
        mode_machine_ = low_state.mode_machine();
    }
}

void G1HardwareInterface::TorsoImuHandler(const void *message)
{
    IMUState_ imu_msg = *(const IMUState_ *)message;

    ImuState torso_imu;
    torso_imu.quat = imu_msg.quaternion();
    torso_imu.rpy = imu_msg.rpy();
    torso_imu.omega = imu_msg.gyroscope();
    torso_imu.acc = imu_msg.accelerometer();
    torso_imu_buffer_.SetData(torso_imu);
}

void G1HardwareInterface::Estimate()
{
    BipedProprioception proprioception = GetBipedProprioceptionFromRawSensorDataHardware(sensor_hw_);



    robot_->UpdateKinematics(proprioception.q(loco_proprio_indices_), proprioception.qdot(loco_proprio_indices_));

    BipedEstimationKinematicsInput biped_estimation_kinematics_input = robot_->ComputeEstimationKinematicsInput();
    est_lin_vel_ = contact_kf_->Update(simple_timer_.ElapsedSinceStart(), sensor_hw_.base_lin_acc, sensor_hw_.base_ang_quat, biped_estimation_kinematics_input);

    proprioception.qdot.head(3) << est_lin_vel_;
    full_proprioception_ = proprioception;

    // Update subsets
    loco_proprioception_ = full_proprioception_.GetIndex(loco_proprio_indices_);
    pd_proprioception_ = full_proprioception_.GetIndex(pd_proprio_indices_);

    std::cout << "proprioception q: " << full_proprioception_.q(loco_proprio_indices_).transpose() << std::endl;
    std::cout << "proprioception qdot: " << full_proprioception_.qdot(loco_proprio_indices_).transpose() << std::endl;
}

BipedProprioception G1HardwareInterface::ReadAndEstimate()
{
    
    simple_timer_.Tick();

    // Get latest state from buffers
    ConvertG1StateToProprioception();
    Estimate();

    Eigen::VectorXf LogData;

    std::vector<Eigen::VectorXd> log_vectors = {est_lin_vel_, full_proprioception_.q, full_proprioception_.qdot};
    LogData = CollectLog(simple_timer_.ElapsedSinceStart(), log_vectors);
    if (logFile_.is_open())
    {
        logFile_.write(reinterpret_cast<char *>(LogData.data()), LogData.size() * sizeof(float));
    }
    return loco_proprioception_;
}

void G1HardwareInterface::ConvertG1StateToProprioception()
{
    const std::shared_ptr<const MotorState> motor_state = motor_state_buffer_.GetData();
    const std::shared_ptr<const ImuState> pelvis_imu = pelvis_imu_buffer_.GetData();
    for (int i = 0; i < total_motor_dof_; ++i)
    {
        sensor_hw_.encoders_pos_pinocchio_order(i) = static_cast<double>(motor_state->q[i]);
        sensor_hw_.encoders_vel_pinocchio_order(i) = static_cast<double>(motor_state->dq[i]);
    }

    sensor_hw_.base_ang_quat.w() = static_cast<double>(pelvis_imu->quat[0]);
    sensor_hw_.base_ang_quat.x() = static_cast<double>(pelvis_imu->quat[1]);
    sensor_hw_.base_ang_quat.y() = static_cast<double>(pelvis_imu->quat[2]);
    sensor_hw_.base_ang_quat.z() = static_cast<double>(pelvis_imu->quat[3]);

    //make sure the quaternion is normalized
    sensor_hw_.base_ang_quat.normalize();
    

    sensor_hw_.base_ang_vel << static_cast<double>(pelvis_imu->omega[0]),
        static_cast<double>(pelvis_imu->omega[1]),
        static_cast<double>(pelvis_imu->omega[2]);

    sensor_hw_.base_lin_acc << static_cast<double>(pelvis_imu->acc[0]),
        static_cast<double>(pelvis_imu->acc[1]),
        static_cast<double>(pelvis_imu->acc[2]);
}

void G1HardwareInterface::SendPacket()
{
    CheckSafeMotorCommands(loco_motor_commands_);
    // Convert final motor commands to G1 format
    ConvertMotorCommandsToG1(final_motor_commands_);

    std::cout << "Final motor commands: " << std::endl;
    std::cout << final_motor_commands_ << std::endl;

    // Send commands to robot
    LowCommandWriter();


    // Update torque in robot model for contact classifier by compute motored joints PD control
    torque_loco_ = loco_motor_commands_.SolveFullTorque(loco_proprioception_.q(loco_proprio_motor_indices_),
                                                        loco_proprioception_.qdot(loco_proprio_motor_indices_));
    robot_->set_computed_torque(torque_loco_);
}

void G1HardwareInterface::ConvertMotorCommandsToG1(const BipedMotorCommands &commands)
{
    MotorCommand mc_tmp;

    // Convert from proprioception space to G1 space
    for (int i = 0; i < total_motor_dof_; ++i)
    {
        //need to cast to float for G1
        mc_tmp.q_target[i] = static_cast<float>(commands.joint_positions(i));
        mc_tmp.dq_target[i] = static_cast<float>(commands.joint_velocities(i));
        mc_tmp.kp[i] = static_cast<float>(commands.joint_kp(i));
        mc_tmp.kd[i] = static_cast<float>(commands.joint_kd(i));
        mc_tmp.tau_ff[i] = static_cast<float>(commands.joint_torques_ff(i));
    }

    // Store in buffer for command writer thread
    motor_command_buffer_.SetData(mc_tmp);
}

void G1HardwareInterface::CheckSafeMotorCommands(BipedMotorCommands &commands)
{
    commands.joint_positions = commands.joint_positions.cwiseMax(q_lower_bound_).cwiseMin(q_upper_bound_);
    ProcessMotorCommands(commands);
}


void G1HardwareInterface::LowCommandWriter()
{
    LowCmd_ dds_low_command;
    dds_low_command.mode_pr() = static_cast<uint8_t>(mode_pr_);
    dds_low_command.mode_machine() = mode_machine_;

    const std::shared_ptr<const MotorCommand> mc = motor_command_buffer_.GetData();
    if (mc)
    {
        
        for (int i = 0; i < total_motor_dof_; i++)
        {
            dds_low_command.motor_cmd().at(i).mode() = g1_motor_mode.at(i); // 1:Enable, 0:Disable
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

void G1HardwareInterface::LowCommandWriterDebug()
{
    LowCmd_ dds_low_command;
    dds_low_command.mode_pr() = static_cast<uint8_t>(mode_pr_);
    dds_low_command.mode_machine() = mode_machine_;

    std::cout << "time: " << simple_timer_.ElapsedSinceStart() << std::endl;
    std::cout << "G1 Low Command Writer Debug:" << std::endl;
    std::cout << final_motor_commands_ << std::endl;

    const std::shared_ptr<const MotorCommand> mc = motor_command_buffer_.GetData();
    if (mc)
    {
        // for (int i = 0; i < total_motor_dof_; i++)
        // {
        //     dds_low_command.motor_cmd().at(i).mode() = 1; // 1:Enable, 0:Disable
        //     dds_low_command.motor_cmd().at(i).tau() = 0.0; //mc->tau_ff.at(i);
        //     dds_low_command.motor_cmd().at(i).q() = 0.0; //mc->q_target.at(i);
        //     dds_low_command.motor_cmd().at(i).dq() = 0.0; //mc->dq_target.at(i);
        //     dds_low_command.motor_cmd().at(i).kp() = 10.0; //mc->kp.at(i);
        //     dds_low_command.motor_cmd().at(i).kd() = 1.0; //1mc->kd.at(i);
        // }

        // dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
        // lowcmd_publisher_->Write(dds_low_command);
    }
}

uint32_t G1HardwareInterface::Crc32Core(uint32_t *ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;

    for (uint32_t i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }
            if (data & xbit)
            {
                CRC32 ^= dwPolynomial;
            }
            xbit >>= 1;
        }
    }
    return CRC32;
}
