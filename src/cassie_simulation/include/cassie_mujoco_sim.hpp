#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "cassie_sensor.hpp"


#include "biped_core/mujoco_sim_base.hpp"

#include <cassie_interface/cassie_out_t.h>
#include <cassie_interface/cassie_user_in_t.h>
#include <cassie_interface/cassiemujoco.h> // mujoco


class CassieMujocoSim : public MujocoSimBase
{
public:
    CassieMujocoSim();
    CassieMujocoSim(const std::string &config_file);
    ~CassieMujocoSim() override;

    void Init(const std::string& config_file) override;
    bool Step(const Eigen::VectorXd& leg_control_input, const Eigen::VectorXd& upper_control_input) override;
    void GetAllJointStateFromSensorMujoco(Eigen::VectorXd& q, Eigen::VectorXd& qdot) override;

    void SimHoldPelvis() override {cassie_sim_hold(sim);}
    void SimReleasePelvis() override {cassie_sim_release(sim);}

    void Close() override;
    bool paused() override {return cassie_vis_paused(vis);}
    double sim_time() override {return *timeMujoP;}


private:



   cassie_sensor sensor_;

   cassie_sim_t *sim;
   cassie_vis_t *vis;
   double *timeMujoP;

   cassie_out_t cassie_out;
   cassie_user_in_t cassie_user_in = {0};

   int step_counter = 0;
   int step_counter_threshold = 0;
};
