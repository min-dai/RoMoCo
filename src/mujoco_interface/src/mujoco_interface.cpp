#include "mujoco_interface/mujoco_interface.hpp"

#include "mujoco/mjvisualize.h"
#include <cctype> // for toupper
const char MujocoInterface::help_content_[] =
    "Alt mouse button\n"
    "UI right hold\n"
    "UI title double-click\n"
    "Space\n"
    "Esc\n"
    "Right arrow\n"
    "Left arrow\n"
    "Down arrow\n"
    "Up arrow\n"
    "Page Up\n"
    "Double-click\n"
    "Right double-click\n"
    "Ctrl Right double-click\n"
    "Scroll, middle drag\n"
    "Left drag\n"
    "[Shift] right drag\n"
    "Ctrl [Shift] drag\n"
    "Ctrl [Shift] right drag";

const char MujocoInterface::help_title_[] =
    "Swap left-right\n"
    "Show UI shortcuts\n"
    "Expand/collapse all  \n"
    "Pause\n"
    "Free camera\n"
    "Step forward\n"
    "Step back\n"
    "Step forward 100\n"
    "Step back 100\n"
    "Select parent\n"
    "Select\n"
    "Center\n"
    "Track camera\n"
    "Zoom\n"
    "View rotate\n"
    "View translate\n"
    "Object rotate\n"
    "Object translate";

MujocoInterface *MujocoInterface::instance_ = nullptr;

MujocoInterface::MujocoInterface()
    : m(nullptr), d(nullptr)
{
    instance_ = this;
}

MujocoInterface::~MujocoInterface()
{
    Close();
    instance_ = nullptr;
}

void MujocoInterface::Sim1StepForward()
{
    mj_step(m, d);
}

bool MujocoInterface::IsWindowOpen() const
{
    return vis_.window && !glfwWindowShouldClose(vis_.window);
}

bool MujocoInterface::Init(const char *modelfile, int width, int height)
{
    char error[1000] = "Could not load binary model";
    m = mj_loadXML(modelfile, nullptr, error, 1000);

    d = mj_makeData(m);

    std::cout << "--------------------------" << std::endl;
    std::cout << "Model file: " << modelfile << std::endl;

    std::cout << "Mujoco Model loaded successfully." << std::endl;

    std::cout << "nq: " << m->nq << std::endl;
    std::cout << "nv: " << m->nv << std::endl;
    std::cout << "nu: " << m->nu << std::endl;
    std::cout << "na: " << m->na << std::endl;
    std::cout << "neq: " << m->neq << std::endl;
    std::cout << "nsensordata: " << m->nsensordata << std::endl;
    std::cout << "--------------------------" << std::endl;

    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    vis_.video_width = width;
    vis_.video_height = height;

    // create window
    vis_.window = glfwCreateWindow(width, height, "Demo", NULL, NULL);
    if (!vis_.window)
    {
        std::cerr << "[ERROR] Failed to create GLFW window\n";
        return false;
    }
    glfwMakeContextCurrent(vis_.window);
    glfwSwapInterval(0);

    // set callbacks
    glfwSetKeyCallback(vis_.window, keyboard);
    glfwSetCursorPosCallback(vis_.window, mouse_move);
    glfwSetMouseButtonCallback(vis_.window, mouse_button);
    glfwSetScrollCallback(vis_.window, scroll);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    mj_forward(m, d); // Compute forward dynamics to populate mjData

    return true;
}

void MujocoInterface::Close()
{
    if (vis_.pipe_video_out)
    {
        StopVideoRecording(); // already frees vis_.frame
    }

    if (vis_.window)
    {
        glfwDestroyWindow(vis_.window);
        vis_.window = nullptr;
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    if (d)
    {
        mj_deleteData(d);
        d = nullptr;
    }
    if (m)
    {
        mj_deleteModel(m);
        m = nullptr;
    }

    // Only call once per app (if you own GLFW lifecycle)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif
    std::cout << "Mujoco simulation closed successfully." << std::endl;
}

std::vector<int> MujocoInterface::GetActuatorIdsByName(const std::vector<std::string> &actuator_names)
{
    std::vector<int> actuator_ids;

    for (int i = 0; i < actuator_names.size(); i++)
    {
        int actuator_id = mj_name2id(m, mjOBJ_ACTUATOR, actuator_names[i].c_str());
        if (actuator_id == -1)
        {
            printf("Actuator with name '%s' not found.\n", actuator_names[i].c_str());
            return actuator_ids;
        }
        actuator_ids.push_back(actuator_id);
    }

    return actuator_ids;
}

std::vector<int> MujocoInterface::GetSensorIdsByName(const std::vector<std::string> &sensor_names)
{
    std::vector<int> sensor_ids;

    for (int i = 0; i < sensor_names.size(); i++)
    {
        int sensor_id = mj_name2id(m, mjOBJ_SENSOR, sensor_names[i].c_str());
        if (sensor_id == -1)
        {
            printf("Sensor with name '%s' not found.\n", sensor_names[i].c_str());
            return sensor_ids;
        }
        sensor_ids.push_back(sensor_id);
    }

    return sensor_ids;
}

std::vector<int> MujocoInterface::GetJointIdsByName(const std::vector<std::string> &joint_names)
{
    std::vector<int> joint_ids;

    for (int i = 0; i < joint_names.size(); i++)
    {
        int joint_id = mj_name2id(m, mjOBJ_JOINT, joint_names[i].c_str());
        if (joint_id == -1)
        {
            printf("Joint with name '%s' not found.\n", joint_names[i].c_str());
            return joint_ids;
        }
        joint_ids.push_back(joint_id);
    }

    return joint_ids;
}

void MujocoInterface::set_1dof_joint_qpos(const Eigen::VectorXd &qpos, const std::vector<int> &joint_ids)
{
    assert(joint_ids.size() == qpos.size());
    for (int i = 0; i < joint_ids.size(); i++)
    {
        int joint_id = joint_ids[i];
        assert(joint_id >= 0 && joint_id < m->njnt);

        // Get correct qpos index
        int qpos_index = m->jnt_qposadr[joint_id];

        // Set the desired joint position
        d->qpos[qpos_index] = qpos[i];
    }
}

void MujocoInterface::set_base_quaternion(const Eigen::Vector4d &quat)
{
    // w,x,y,z
    d->qpos[3] = quat[0];
    d->qpos[4] = quat[1];
    d->qpos[5] = quat[2];
    d->qpos[6] = quat[3];
}

void MujocoInterface::set_base_pos(const Eigen::Vector3d &pos)
{
    d->qpos[0] = pos[0];
    d->qpos[1] = pos[1];
    d->qpos[2] = pos[2];
}

void MujocoInterface::set_qpos(const Eigen::VectorXd &qpos)
{
    assert(qpos.size() == m->nq);
    for (int i = 0; i < qpos.size(); i++)
    {
        d->qpos[i] = qpos[i];
    }
}

void MujocoInterface::set_zero_qvel()
{
    for (int i = 0; i < m->nv; i++)
    {
        d->qvel[i] = 0;
    }
}

void MujocoInterface::set_zero_qacc()
{
    for (int i = 0; i < m->nv; i++)
    {
        d->qacc[i] = 0;
    }
}

void MujocoInterface::keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    if (act == GLFW_PRESS)
    {

        switch (key)
        {
        // backspace: reset simulation
        case GLFW_KEY_BACKSPACE:
        {
            mj_resetData(instance_->m, instance_->d);
            mj_forward(instance_->m, instance_->d);
        }
        break;
        case GLFW_KEY_SPACE:
        {
            instance_->settings_.paused = !instance_->settings_.paused;
            instance_->settings_.paused ? printf("Paused\n") : printf("Running\n");
        }
        break;
        case GLFW_KEY_F1:
        {
            instance_->settings_.show_help = !instance_->settings_.show_help;
        }
        break;
        case GLFW_KEY_F2:
        {
            instance_->settings_.show_info = !instance_->settings_.show_info;
        }
        break;
        case GLFW_KEY_P:
        {
            // set camera to attach to the pelvis on sagittal plane
            instance_->cam.type = mjCAMERA_TRACKING;
            instance_->cam.trackbodyid = mj_name2id(instance_->m, mjOBJ_BODY, "pelvis");
            instance_->cam.fixedcamid = -1;
            instance_->cam.distance = 2.0;
            instance_->cam.azimuth = 90;
            instance_->cam.elevation = -20;
        }
        break;
        case GLFW_KEY_L:
        {
            // set camera to attach to the pelvis on lateral plane
            instance_->cam.type = mjCAMERA_TRACKING;
            instance_->cam.trackbodyid = mj_name2id(instance_->m, mjOBJ_BODY, "pelvis");
            instance_->cam.fixedcamid = -1;
            instance_->cam.distance = 2.0;
            instance_->cam.azimuth = 180;
            instance_->cam.elevation = -20;
        }
        break;
        case GLFW_KEY_V:
        {
            // set camera to attach to the pelvis on diagonal view
            instance_->cam.type = mjCAMERA_TRACKING;
            instance_->cam.trackbodyid = mj_name2id(instance_->m, mjOBJ_BODY, "pelvis");
            instance_->cam.fixedcamid = -1;
            instance_->cam.distance = 2.0;
            instance_->cam.azimuth = 140;
            instance_->cam.elevation = -20;
        }
        break;
        case GLFW_KEY_ESCAPE:
        {
            instance_->cam.type = mjCAMERA_FREE;
        }
        break;
        }

        // Handle MuJoCo visual toggle keys based on mjVISSTRING shortcut mapping
        for (int i = 0; i < mjNVISFLAG; ++i)
        {
            const char shortcut = mjVISSTRING[i][2][0];
            if (shortcut && key == static_cast<int>(toupper(shortcut)))
            {
                instance_->opt.flags[i] ^= 1;
                std::cout << "[VISUAL] Toggled " << mjVISSTRING[i][0] << ": "
                          << (instance_->opt.flags[i] ? "ON" : "OFF") << std::endl;
                return;
            }
        }
        // Handle MuJoCo rendering keys
        for (int i = 0; i < mjNRNDFLAG; ++i)
        {
            const char shortcut = mjRNDSTRING[i][2][0];
            if (shortcut && key == static_cast<int>(toupper(shortcut)))
            {
                instance_->opt.flags[i] ^= 1;
                std::cout << "[RENDER] Toggled " << mjRNDSTRING[i][0] << ": "
                          << (instance_->opt.flags[i] ? "ON" : "OFF") << std::endl;
                return;
            }
        }
    }
}

void MujocoInterface::mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    instance_->vis_.button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    instance_->vis_.button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    instance_->vis_.button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    glfwGetCursorPos(window, &instance_->vis_.lastx, &instance_->vis_.lasty);
}

void MujocoInterface::mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    if (!instance_->vis_.button_left && !instance_->vis_.button_middle && !instance_->vis_.button_right)
        return;

    double dx = xpos - instance_->vis_.lastx;
    double dy = ypos - instance_->vis_.lasty;
    instance_->vis_.lastx = xpos;
    instance_->vis_.lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (instance_->vis_.button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (instance_->vis_.button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    mjv_moveCamera(instance_->m, action, dx / height, dy / height, &instance_->scn, &instance_->cam);
}

void MujocoInterface::scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    mjv_moveCamera(instance_->m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &instance_->scn, &instance_->cam);
}

void MujocoInterface::Render()
{
    vis_.window = glfwGetCurrentContext();

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(vis_.window, &viewport.width, &viewport.height);

    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    if (settings_.show_help)
    {
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, help_title_, help_content_, &con);
    }
    if (settings_.show_info)
    {
        char buf[1024];
        char str_slow[20];
        if (settings_.slowmotion)
        {
            strcpy(str_slow, "(10x slowdown)");
        }
        else
        {
            strcpy(str_slow, "");
        }
        char str_paused[50];
        if (settings_.paused)
        {
            strcpy(str_paused, "\nPaused");
        }
        else
        {
            strcpy(str_paused, "\nRunning");
        }
        strcat(str_paused, "\nTime:");
        char status[50];
        sprintf(status, "\n\n%.2f", d->time);
        strcpy(buf, str_slow);
        strcat(buf, status);
        // status = str_slow * status

        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport,
                    str_paused,
                    buf, &con);
    }

    glfwSwapBuffers(vis_.window);
    glfwPollEvents();
}

void MujocoInterface::UpdateControlInput(const Eigen::VectorXd &u, const std::vector<int> &actuator_ids)
{
    assert(u.size() == actuator_ids.size());

    for (int i = 0; i < u.size(); i++)
    {
        if (actuator_ids[i] >= 0 && actuator_ids[i] < m->nu)
        {
            d->ctrl[actuator_ids[i]] = u(i);
        }
        else
        {
            std::cerr << "Invalid actuator ID: " << actuator_ids[i] << std::endl;
        }
    }
}

std::vector<double> MujocoInterface::GetSensorDataById(int sensor_id)
{
    assert(sensor_id >= 0 && sensor_id < m->nsensor);
    // Get the address of the sensor data
    int sensor_adr = m->sensor_adr[sensor_id];
    int sensor_dim = m->sensor_dim[sensor_id];

    std::vector<double> sensor_data(sensor_dim);

    // Populate the vector with the sensor data
    for (int i = 0; i < sensor_dim; i++)
    {
        sensor_data[i] = d->sensordata[sensor_adr + i];
    }

    return sensor_data;
}

Eigen::VectorXd MujocoInterface::GetSensorDataByIds(const std::vector<int> &sensor_ids)
{
    std::vector<double> sensor_data;

    for (int i = 0; i < sensor_ids.size(); i++)
    {
        std::vector<double> sensor_data_i = GetSensorDataById(sensor_ids[i]);
        sensor_data.insert(sensor_data.end(), sensor_data_i.begin(), sensor_data_i.end());
    }

    return Eigen::Map<Eigen::VectorXd>(sensor_data.data(), sensor_data.size());
}

Eigen::VectorXd MujocoInterface::GetJointVelocitiesByIds(const std::vector<int> &joint_ids)
{
    std::vector<double> joint_velocity;

    for (int i = 0; i < joint_ids.size(); i++)
    {
        int joint_id = joint_ids[i];
        assert(joint_id >= 0 && joint_id < m->njnt);
        int joint_adr = m->jnt_dofadr[joint_id];

        joint_velocity.push_back(d->qvel[joint_adr]);
    }

    return Eigen::Map<Eigen::VectorXd>(joint_velocity.data(), joint_velocity.size());
}

Eigen::VectorXd MujocoInterface::GetJointPositionsByIds(const std::vector<int> &joint_ids)
{
    std::vector<double> joint_position;

    for (int i = 0; i < joint_ids.size(); i++)
    {
        int joint_id = joint_ids[i];
        assert(joint_id >= 0 && joint_id < m->njnt);
        int joint_adr = m->jnt_qposadr[joint_id];

        joint_position.push_back(d->qpos[joint_adr]);
    }

    return Eigen::Map<Eigen::VectorXd>(joint_position.data(), joint_position.size());
}

void MujocoInterface::StartVideoRecording(const char *filename, int fps)
{

    char ffmpeg_cmd[1000] = "ffmpeg -hide_banner -loglevel error -y -f rawvideo -vcodec rawvideo -pix_fmt rgb24 -s ";
    char integer_string[32];

    sprintf(integer_string, "%d", vis_.video_width); // Convert and write widthxheight
    strcat(ffmpeg_cmd, integer_string);
    strcat(ffmpeg_cmd, "x");
    sprintf(integer_string, "%d", vis_.video_height);
    strcat(ffmpeg_cmd, integer_string);

    strcat(ffmpeg_cmd, " -r "); // Frame Rate
    sprintf(integer_string, "%d", fps);
    strcat(ffmpeg_cmd, integer_string); // Frame Rate

    strcat(ffmpeg_cmd, " -i - -f mp4 -an -c:v libx264 -preset slow -crf 17 -vf \"vflip\" ");
    strcat(ffmpeg_cmd, filename);
    if (strstr(filename, ".mp4") == NULL) // add a .mp4 if you forgot
        strcat(ffmpeg_cmd, ".mp4");

    vis_.frame = (unsigned char *)malloc(3 * vis_.video_width * vis_.video_height);

    vis_.pipe_video_out = popen(ffmpeg_cmd, "w");
}

void MujocoInterface::RecordVideoFrame()
{
    mjrRect viewport = {0, 0, 0, 0};

    glfwGetFramebufferSize(vis_.window, &viewport.width, &viewport.height);

    mjr_readPixels(vis_.frame, NULL, viewport, &con);
    // Write frame to output pipe
    fwrite(vis_.frame, 1, vis_.video_width * vis_.video_height * 3, vis_.pipe_video_out);
}

void MujocoInterface::StopVideoRecording()
{
    if (vis_.pipe_video_out != NULL)
    {
        fflush(vis_.pipe_video_out);
        pclose(vis_.pipe_video_out);
        vis_.pipe_video_out = NULL;
        free(vis_.frame);
    }
}

void MujocoInterface::SimHoldPelvis()
{
    // Hold translational DOFs (index 0–2)
    for (int i = 0; i < 3; ++i)
    {
        m->dof_damping[i] = 1e6;
        m->qpos_spring[i] = d->qpos[i];
    }

    // Hold rotational DOFs via quaternion (qpos[3:7])
    for (int i = 3; i < 7; ++i)
    {
        m->qpos_spring[i] = d->qpos[i];
    }

    // Add damping to rotational DOFs (index 3–5)
    for (int i = 3; i < 6; ++i)
    {
        m->dof_damping[i] = 1e5;
    }
}

void MujocoInterface::SimReleasePelvis()
{
    // Release translational DOFs
    for (int i = 0; i < 3; ++i)
    {
        m->dof_damping[i] = 0.0;
        m->qpos_spring[i] = 0.0;
    }

    // Release rotational DOFs (quaternion part of qpos[3:7])
    for (int i = 3; i < 7; ++i)
    {
        m->qpos_spring[i] = 0.0;
    }

    // Damping for rotational DoFs (indices 3–5)
    for (int i = 3; i < 6; ++i)
    {
        m->dof_damping[i] = 0.0;
    }
}
