#include <cassie_mujoco_sim.hpp>
#include "biped_utils/yaml_parser.hpp"
CassieMujocoSim::CassieMujocoSim() : sim(nullptr), vis(nullptr), timeMujoP(nullptr)
{
}

CassieMujocoSim::CassieMujocoSim(const std::string &config_file) : sim(nullptr), vis(nullptr), timeMujoP(nullptr)
{
    Init(config_file);
}

CassieMujocoSim::~CassieMujocoSim()
{
    Close();
}

void CassieMujocoSim::Init(const std::string& config_folder)
{
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

   




    step_counter_threshold = std::floor(1 / 0.0005 / double(videoSetting.video_fps));

    

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
    sensor_.cassie_out = cassie_out;


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
        sensor_.cassie_out = cassie_out;

    }


    step_counter++;
    if (step_counter >= step_counter_threshold)
    {
        cassie_vis_draw(vis, sim);
        step_counter = 0;
        if (videoSetting.record_video && !cassie_vis_paused(vis))
        {
            cassie_vis_record_frame(vis);
        }
    }
    return 1;
}

void CassieMujocoSim::GetAllJointStateFromSensorMujoco(Eigen::VectorXd &q, Eigen::VectorXd &qdot)
{
    getAllJointStateFromSensorMujoco(sensor_, q, qdot);
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
