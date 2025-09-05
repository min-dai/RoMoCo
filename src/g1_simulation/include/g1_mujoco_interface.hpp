#ifndef G1_MUJOCO_SIM_HPP
#define G1_MUJOCO_SIM_HPP

#include "mujoco_sim/mujoco_sim.hpp"
#include "biped_core/mujoco_interface_base.hpp"


#include "biped_core/robot_base_pinocchio.hpp"
#include "biped_core/contact_kf.hpp"




class G1MujocoInterface : public MujocoInterfaceBase
{
public:
   G1MujocoInterface();
   G1MujocoInterface(const std::string &config_file, const std::string &log_path);
   G1MujocoInterface(const std::string &config_file, const std::string &log_path, std::unique_ptr<RobotBasePinocchio> robot);
   ~G1MujocoInterface() override;


   BipedProprioception ReadAndEstimate() override;
   void SendPacket() override;

   bool Step(const Eigen::VectorXd &leg_control_input, const Eigen::VectorXd &upper_control_input) override;

   void SimHoldPelvis() override { mujoco_.SimHoldPelvis(); }
   void SimReleasePelvis() override { mujoco_.SimReleasePelvis(); }


   bool paused() const override { return mujoco_.paused(); }
   double sim_time() const override { return mujoco_.time(); }

private:
   void Init(const std::string &config_folder, const std::string &log_path) override;


   MujocoSim mujoco_;

   void UpdateMujocoTrueSensorData();

   void HandleRendering();

   void Estimate();

   void set_base_pos(const Eigen::Vector3d &pos) { mujoco_.set_base_pos(pos); }
   void set_base_quaternion(const Eigen::Vector4d &quat) { mujoco_.set_base_quaternion(quat); }

   const std::vector<std::string> gyro_name = {"imu-pelvis-angular-velocity"};
   const std::vector<std::string> accelerometer_name = {"imu-pelvis-linear-acceleration"};
   const std::vector<std::string> framequat_name = {"imu-pelvis-orientation"};

   

   
   // for joint position and velocities
   std::vector<int> all_encoder_mj_ids_pinocchio_order; // Joint IDs

   // for leg and upper body control
   std::vector<int> locomotion_actuator_mj_ids_;   // Actuator IDs
   std::vector<int> locked_actuator_mj_ids_; // Actuator IDs

   // for sensor data
   std::vector<int> gyro_mj_ids;          // IMU sensor IDs
   std::vector<int> accelerometer_mj_ids; // Accelerometer sensor IDs
   std::vector<int> framequat_mj_ids;     // Frame quaternion sensor IDs


   // for testing
   void UpdateHardwareSensorData();
   std::unique_ptr<RobotBasePinocchio> robot_;
   std::unique_ptr<ContactKf> contact_kf_;
   Eigen::Vector3d true_lin_vel_;
   Eigen::Vector3d est_lin_vel_;


};

#endif // G1_MUJOCO_SIM_HPP