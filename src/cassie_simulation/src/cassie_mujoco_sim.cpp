#include <cassie_mujoco_sim.hpp>
#include "biped_utils/yaml_parser.hpp"
CassieMujocoSim::CassieMujocoSim() : sim(nullptr), vis(nullptr), timeMujoP(nullptr)
{
}

CassieMujocoSim::CassieMujocoSim(const std::string &config_file, const std::string &log_path) : sim(nullptr), vis(nullptr), timeMujoP(nullptr)
{
    Init(config_file, log_path);
}

CassieMujocoSim::~CassieMujocoSim()
{
    Close();
}

void CassieMujocoSim::Init(const std::string &config_folder, const std::string &log_path)
{
    total_motor_dof_ = 10;
    total_proprio_dof_ = 18;
    loco_motor_indices_ = {0, 1, 2, 3, 5, 6, 7, 8, 9, 11};
    loco_proprio_indices_ = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
    InitMotorCommands();
    InitProprioception();

    std::string config_file = config_folder + "/mujoco_config.yaml";
    YAMLParser yaml_parser(config_file);

    std::string model_name = yaml_parser.get_string("mujoco_settings/model_name");
    videoSetting.InitVideoSetting(config_file);

    char modelfile[500] = "";
    strcat(modelfile, config_folder.c_str());
    strcat(modelfile, "/../model_files/");
    strcat(modelfile, model_name.c_str());

    sim = cassie_sim_init(modelfile, false);
    timeMujoP = cassie_sim_time(sim);
    vis = cassie_vis_init(sim, modelfile, false);

    if (videoSetting.record_video)
    {
        cassie_vis_init_recording(vis, videoSetting.video_name.c_str(), videoSetting.video_width, videoSetting.video_height, videoSetting.video_fps);
        std::cout << "Recording video..." << std::endl;
    }

    for (int i = 0; i < 10; i++)
    {
        cassie_user_in.torque[i] = 0.;
    }

    cassie_sim_step(sim, &cassie_out, &cassie_user_in);

    std::cout << "Cassie Mujoco simulation initialized successfully." << std::endl;
}

bool CassieMujocoSim::Step(const Eigen::VectorXd &leg_control_input, const Eigen::VectorXd &upper_control_input)
{
    if (!cassie_vis_paused(vis))
    {

        for (int i = 0; i < leg_control_input.size(); i++)
        {
            cassie_user_in.torque[i] = leg_control_input(i);
        }

        cassie_sim_step(sim, &cassie_out, &cassie_user_in);

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

    render_loop_counter_++;
    if (render_loop_counter_ % render_loop_counter_threshold_ == 0)
    {
        cassie_vis_draw(vis, sim);

        if (videoSetting.record_video && !cassie_vis_paused(vis))
        {
            cassie_vis_record_frame(vis);
        }
    }
    return 1;
}

void CassieMujocoSim::Close()
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
