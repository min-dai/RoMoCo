#include <cassie_mujoco_interface.hpp>
#include "biped_core/prep_proprioception.hpp"

CassieMujocoInterface::CassieMujocoInterface() : sim(nullptr), vis(nullptr), timeMujoP(nullptr)
{
}

CassieMujocoInterface::CassieMujocoInterface(const std::string &config_folder, const std::string &log_path) : sim(nullptr), vis(nullptr), timeMujoP(nullptr)
{
    Init(config_folder, log_path);
}

CassieMujocoInterface::CassieMujocoInterface(const std::string &config_folder, const std::string &log_path, std::unique_ptr<RobotBasePinocchio> robot)
    : robot_(std::move(robot)), sim(nullptr), vis(nullptr), timeMujoP(nullptr), contact_kf_(std::make_unique<ContactKf>(config_folder, 6))
{
    Init(config_folder, log_path);
}

CassieMujocoInterface::~CassieMujocoInterface()
{
    Close();
}

void CassieMujocoInterface::Init(const std::string &config_folder, const std::string &log_path)
{
    InitLogFile(log_path, true);

    all_encoder_names_pinocchio_order_ = {"LeftHipRoll", "LeftHipYaw", "LeftHipPitch", "LeftKneePitch", "LeftTarsusPitch", "LeftFootPitch",
                                          "RightHipRoll", "RightHipYaw", "RightHipPitch", "RightKneePitch", "RightTarsusPitch", "RightFootPitch"};

    InitDofAndIndicesFromConfigFile(config_folder);
    InitMotorCommands();
    InitProprioception();

    std::string config_file = config_folder + "/mujoco_config.yaml";
    YAMLParser yaml_parser(config_file);

    std::string model_name = yaml_parser.get_string("mujoco_settings/model_name");
    videoSetting.InitVideoSetting(config_file);

    const std::string model_path = config_folder + "/../model_files/" + model_name;

    sim = cassie_sim_init(model_path.c_str(), false);
    timeMujoP = cassie_sim_time(sim);
    vis = cassie_vis_init(sim, model_path.c_str(), false);
    std::cout << "Mujoco model loaded: " << model_name << std::endl;

    if (videoSetting.record_video)
    {
        cassie_vis_init_recording(vis, videoSetting.video_name.c_str(), videoSetting.video_width, videoSetting.video_height, videoSetting.video_fps);
        std::cout << "Recording video..." << std::endl;
    }

    for (int i = 0; i < 10; i++)
    {
        cassie_user_in.torque[i] = 0.;
    }

    sensor_.encoders_pos_pinocchio_order.resize(all_encoder_names_pinocchio_order_.size());
    sensor_.encoders_vel_pinocchio_order.resize(all_encoder_names_pinocchio_order_.size());
    sensor_hw_.encoders_pos_pinocchio_order.resize(all_encoder_names_pinocchio_order_.size());
    sensor_hw_.encoders_vel_pinocchio_order.resize(all_encoder_names_pinocchio_order_.size());

    cassie_sim_step(sim, &cassie_out, &cassie_user_in);

    torque_loco_ = Eigen::VectorXd::Zero(loco_motor_dof_);
    torque_pd_ = Eigen::VectorXd::Zero(pd_motor_dof_);

    if (robot_ != nullptr)
    {
        robot_->SetComputedTorque(torque_loco_);
        UpdateHardwareSensorData();
        robot_->ReconfigureContactClassifier(config_folder);
        std::cout << "Initial HW Sensor Data :" << sensor_hw_ << std::endl;
    }
    else
    {
        UpdateMujocoTrueSensorData();
        std::cout << "Initial True Sensor Data :" << sensor_ << std::endl;
    }

    std::cout << "Cassie Mujoco simulation initialized successfully." << std::endl;
}

bool CassieMujocoInterface::Step(const Eigen::VectorXd &leg_control_input, const Eigen::VectorXd &upper_control_input)
{
    if (!cassie_vis_paused(vis))
    {

        for (int i = 0; i < leg_control_input.size(); i++)
        {
            cassie_user_in.torque[i] = leg_control_input(i);
        }

        cassie_sim_step(sim, &cassie_out, &cassie_user_in);

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

void CassieMujocoInterface::Close()
{
    if (videoSetting.record_video)
    {
        cassie_vis_close_recording(vis);
    }
    if (sim)
    {
        cassie_sim_free(sim);
        sim = nullptr;
    }
    if (vis)
    {
        cassie_vis_free(vis);
        vis = nullptr;
    }
    std::cout << "Cassie Mujoco simulation closed successfully." << std::endl;
}

void CassieMujocoInterface::UpdateMujocoTrueSensorData()
{
    double *qvel;
    qvel = cassie_sim_qvel(sim);

    double *qpos;
    qpos = cassie_sim_qpos(sim);

    sensor_.base_lin_pos << qpos[0], qpos[1], qpos[2];
    sensor_.base_lin_vel << qvel[0], qvel[1], qvel[2];

    sensor_.base_ang_quat.w() = cassie_out.pelvis.vectorNav.orientation[0];
    sensor_.base_ang_quat.x() = cassie_out.pelvis.vectorNav.orientation[1];
    sensor_.base_ang_quat.y() = cassie_out.pelvis.vectorNav.orientation[2];
    sensor_.base_ang_quat.z() = cassie_out.pelvis.vectorNav.orientation[3];

    sensor_.base_ang_vel << cassie_out.pelvis.vectorNav.angularVelocity[0],
        cassie_out.pelvis.vectorNav.angularVelocity[1],
        cassie_out.pelvis.vectorNav.angularVelocity[2];

    sensor_.encoders_pos_pinocchio_order << cassie_out.leftLeg.hipRollDrive.position,
        cassie_out.leftLeg.hipYawDrive.position,
        cassie_out.leftLeg.hipPitchDrive.position,
        cassie_out.leftLeg.kneeDrive.position,
        cassie_out.leftLeg.tarsusJoint.position,
        cassie_out.leftLeg.footJoint.position,
        cassie_out.rightLeg.hipRollDrive.position,
        cassie_out.rightLeg.hipYawDrive.position,
        cassie_out.rightLeg.hipPitchDrive.position,
        cassie_out.rightLeg.kneeDrive.position,
        cassie_out.rightLeg.tarsusJoint.position,
        cassie_out.rightLeg.footJoint.position;
    sensor_.encoders_vel_pinocchio_order << cassie_out.leftLeg.hipRollDrive.velocity,
        cassie_out.leftLeg.hipYawDrive.velocity,
        cassie_out.leftLeg.hipPitchDrive.velocity,
        cassie_out.leftLeg.kneeDrive.velocity,
        cassie_out.leftLeg.tarsusJoint.velocity,
        cassie_out.leftLeg.footJoint.velocity,
        cassie_out.rightLeg.hipRollDrive.velocity,
        cassie_out.rightLeg.hipYawDrive.velocity,
        cassie_out.rightLeg.hipPitchDrive.velocity,
        cassie_out.rightLeg.kneeDrive.velocity,
        cassie_out.rightLeg.tarsusJoint.velocity,
        cassie_out.rightLeg.footJoint.velocity;
}

void CassieMujocoInterface::UpdateHardwareSensorData()
{
    sensor_hw_.encoders_pos_pinocchio_order << cassie_out.leftLeg.hipRollDrive.position,
        cassie_out.leftLeg.hipYawDrive.position,
        cassie_out.leftLeg.hipPitchDrive.position,
        cassie_out.leftLeg.kneeDrive.position,
        cassie_out.leftLeg.tarsusJoint.position,
        cassie_out.leftLeg.footJoint.position,
        cassie_out.rightLeg.hipRollDrive.position,
        cassie_out.rightLeg.hipYawDrive.position,
        cassie_out.rightLeg.hipPitchDrive.position,
        cassie_out.rightLeg.kneeDrive.position,
        cassie_out.rightLeg.tarsusJoint.position,
        cassie_out.rightLeg.footJoint.position;
    sensor_hw_.encoders_vel_pinocchio_order << cassie_out.leftLeg.hipRollDrive.velocity,
        cassie_out.leftLeg.hipYawDrive.velocity,
        cassie_out.leftLeg.hipPitchDrive.velocity,
        cassie_out.leftLeg.kneeDrive.velocity,
        cassie_out.leftLeg.tarsusJoint.velocity,
        cassie_out.leftLeg.footJoint.velocity,
        cassie_out.rightLeg.hipRollDrive.velocity,
        cassie_out.rightLeg.hipYawDrive.velocity,
        cassie_out.rightLeg.hipPitchDrive.velocity,
        cassie_out.rightLeg.kneeDrive.velocity,
        cassie_out.rightLeg.tarsusJoint.velocity,
        cassie_out.rightLeg.footJoint.velocity;
    sensor_hw_.base_ang_quat.w() = cassie_out.pelvis.vectorNav.orientation[0];
    sensor_hw_.base_ang_quat.x() = cassie_out.pelvis.vectorNav.orientation[1];
    sensor_hw_.base_ang_quat.y() = cassie_out.pelvis.vectorNav.orientation[2];
    sensor_hw_.base_ang_quat.z() = cassie_out.pelvis.vectorNav.orientation[3];

    sensor_hw_.base_ang_vel << cassie_out.pelvis.vectorNav.angularVelocity[0],
        cassie_out.pelvis.vectorNav.angularVelocity[1],
        cassie_out.pelvis.vectorNav.angularVelocity[2];
    sensor_hw_.base_lin_acc << cassie_out.pelvis.vectorNav.linearAcceleration[0],
        cassie_out.pelvis.vectorNav.linearAcceleration[1],
        cassie_out.pelvis.vectorNav.linearAcceleration[2];
}

void CassieMujocoInterface::Estimate()
{
    BipedProprioception proprioception = GetBipedProprioceptionFromRawSensorDataHardware(sensor_hw_);

    robot_->UpdateKinematics(proprioception.q(loco_proprio_indices_), proprioception.qdot(loco_proprio_indices_));

    BipedEstimationKinematicsInput biped_estimation_kinematics_input = robot_->ComputeEstimationKinematicsInput();

    std::cout << "estimation input" << std::endl;
    std::cout << biped_estimation_kinematics_input << std::endl;
    est_lin_vel_ = contact_kf_->Update(sim_time(), sensor_hw_.base_lin_acc, sensor_hw_.base_ang_quat, biped_estimation_kinematics_input);

    proprioception.qdot.head(3) << est_lin_vel_;

    std::cout << "proprioception" << std::endl;
    std::cout << proprioception << std::endl;

    if (torque_loco_.hasNaN())
    {
        std::cerr << "Error: torque_loco_ contains NaN values!" << std::endl;
        // terminate sim
        exit(EXIT_FAILURE);
    }

    full_proprioception_ = proprioception;
}

void CassieMujocoInterface::ReadAndEstimate()
{
    if (robot_ != nullptr)
    {
        Estimate();
        loco_proprioception_ = full_proprioception_.GetIndex(loco_proprio_indices_);
        pd_proprioception_ = full_proprioception_.GetIndex(pd_proprio_indices_);
    }
    else
    {
        MujocoInterfaceBase::ReadAndEstimate();
    }
}
void CassieMujocoInterface::SendPacket()
{
    MujocoInterfaceBase::SendPacket();
    std::cout << "Loco Torque Command: " << torque_loco_.transpose() << std::endl;
    if (robot_ != nullptr)
    {
        robot_->SetComputedTorque(torque_loco_);
    }
}

void CassieMujocoInterface::HandleRendering()
{
    render_loop_counter_++;
    if (render_loop_counter_ % render_loop_counter_threshold_ == 0)
    {
        cassie_vis_draw(vis, sim);

        if (videoSetting.record_video && !cassie_vis_paused(vis))
        {
            cassie_vis_record_frame(vis);
        }
    }
}
