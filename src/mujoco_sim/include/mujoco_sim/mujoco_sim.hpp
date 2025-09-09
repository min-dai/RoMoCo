#ifndef MUJOCOINTERFACE_H
#define MUJOCOINTERFACE_H

#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include <iostream>
#include <cstring>

#include <Eigen/Dense>

#include <vector>

/**
 * @class MujocoSim
 * @brief A class for simulating and visualizing physics models using MuJoCo.
 *
 * MujocoSim provides an interface for initializing, running, and interacting with MuJoCo simulations.
 * It supports rendering, video recording, sensor and actuator data access, and manipulation of simulation state.
 *
 * Main Features:
 * - Initialization and closure of MuJoCo simulation.
 * - Rendering and visualization controls.
 * - Simulation stepping and control input updates.
 * - Pelvis hold/release functionality for humanoid models.
 * - Video recording and frame capture.
 * - Access to actuator, sensor, and joint IDs by name.
 * - Retrieval of sensor data, joint positions, and velocities.
 * - Direct access and manipulation of joint and base states.
 * - Window and user interaction management via GLFW.
 *
 * Usage:
 * 1. Create an instance of MujocoSim.
 * 2. Initialize with a model file and rendering options.
 * 3. Use provided methods to interact with the simulation.
 * 4. Close the simulation when done.
 *
 * Thread Safety:
 * - Not guaranteed. User must ensure thread safety if used in multi-threaded contexts.
 *
 * Dependencies:
 * - MuJoCo library
 * - Eigen library
 * - GLFW library
 */
class MujocoSim
{

public:
    MujocoSim();
    ~MujocoSim();

    bool Init(const char *modelfile, int width, int height, bool headless = false);
    void Close();

    void Render();

    void Sim1StepForward();
    void UpdateControlInput(const Eigen::VectorXd &u, const std::vector<int> &actuator_ids);

    void SimHoldPelvis();
    void SimReleasePelvis();

    void StartVideoRecording(const char *filename, int fps);
    void StopVideoRecording();
    void RecordVideoFrame();

    inline bool paused() const { return settings_.paused; }
    inline double time() const { return d->time; }
    inline double timestep() const { return m->opt.timestep; }

    std::vector<int> GetActuatorIdsByName(const std::vector<std::string> &actuator_names);
    std::vector<int> GetSensorIdsByName(const std::vector<std::string> &sensor_names);
    std::vector<int> GetJointIdsByName(const std::vector<std::string> &joint_names);

    Eigen::VectorXd GetSensorDataByIds(const std::vector<int> &sensor_ids);
    Eigen::VectorXd GetJointPositionsByIds(const std::vector<int> &joint_ids);
    Eigen::VectorXd GetJointVelocitiesByIds(const std::vector<int> &joint_ids);

    double *qpos() const { return d->qpos; }
    double *qvel() const { return d->qvel; }

    void set_1dof_joint_qpos(const Eigen::VectorXd &qpos, const std::vector<int> &joint_ids);
    void set_base_quaternion(const Eigen::Vector4d &quat);
    void set_base_pos(const Eigen::Vector3d &pos);
    void set_qpos(const Eigen::VectorXd &qpos);
    void set_zero_qvel();
    void set_zero_qacc();

    bool IsWindowOpen() const;

private:
    // MuJoCo data structures
    mjModel *m;
    mjData *d;
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;

    struct Settings
    {
        int width = 1000;
        int height = 1000;
        int frame_skip = 1;
        int nsubsteps = 1;
        bool show_sensor = false;
        bool slowmotion = false;
        bool show_info = true;
        bool show_help = false;
        bool paused = true;

    } settings_;

    struct visualization
    {
        // mouse interaction
        bool button_left = false;
        bool button_middle = false;
        bool button_right = false;
        double lastx = 0.;
        double lasty = 0.;
        bool showfullscreen = false;
        bool showsensor = false;
        bool slowmotion = false;
        bool showinfo = false;

        // GLFW  handle
        GLFWwindow *window;

        FILE *pipe_video_out;
        int video_width;
        int video_height;
        unsigned char *frame;

    } vis_;

    // GLFW callbacks
    static void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods);
    static void mouse_button(GLFWwindow *window, int button, int act, int mods);
    static void mouse_move(GLFWwindow *window, double xpos, double ypos);
    static void scroll(GLFWwindow *window, double xoffset, double yoffset);

    static MujocoSim *instance_; // For managing callbacks

    std::vector<double> GetSensorDataById(int sensor_id);

private:
    // help strings
    static const char help_content_[];
    static const char help_title_[];
};

#endif // MUJOCOINTERFACE_H
