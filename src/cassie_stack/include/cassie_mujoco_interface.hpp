#ifndef CASSIE_MUJOCO_INTERFACE_HPP
#define CASSIE_MUJOCO_INTERFACE_HPP

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>


#include "romoco_core/mujoco_interface_base.hpp"
#include "romoco_core/contact_kf.hpp"
#include <cassie_interface/cassie_out_t.h>
#include <cassie_interface/cassie_user_in_t.h>
#include <cassie_interface/cassiemujoco.h> // mujoco


class CassieMujocoInterface : public MujocoInterfaceBase
{
public:
    CassieMujocoInterface();
    CassieMujocoInterface(const std::string &config_file, const std::string &log_path);
    CassieMujocoInterface(const std::string &config_file, const std::string &log_path, std::unique_ptr<RobotBasePinocchio> robot);
    ~CassieMujocoInterface() override;

    BipedProprioception ReadAndEstimate() override;
    void SendPacket() override;


    
    bool Step(const Eigen::VectorXd& leg_control_input, const Eigen::VectorXd& upper_control_input) override;

    void SimHoldPelvis() override {cassie_sim_hold(sim);}
    void SimReleasePelvis() override {cassie_sim_release(sim);}

    
    bool paused() const override {return cassie_vis_paused(vis);}
    double sim_time() const override {return *timeMujoP;}


private:
    void Init(const std::string& config_file, const std::string& log_path) override;

    void Close();

    void UpdateMujocoTrueSensorData();

   void HandleRendering();

   void Estimate();


   cassie_sim_t *sim;
   cassie_vis_t *vis;
   double *timeMujoP;

   cassie_out_t cassie_out;
   cassie_user_in_t cassie_user_in = {0};

   // for estimation
   void UpdateHardwareSensorData();
   std::unique_ptr<RobotBasePinocchio> robot_;
   std::unique_ptr<ContactKf> contact_kf_;
   Eigen::Vector3d true_lin_vel_;
   Eigen::Vector3d est_lin_vel_;

};

#endif // CASSIE_MUJOCO_INTERFACE_HPP