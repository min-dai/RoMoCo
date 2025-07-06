#ifndef H1_MUJOCO_SIM_HPP
#define H1_MUJOCO_SIM_HPP

#include "mujoco_interface/mujoco_interface.hpp"
#include "biped_core/mujoco_sim_base.hpp"
#include "h1_sensor.hpp"

class H1MujocoSim : public MujocoSimBase
{
public:
   H1MujocoSim();
   H1MujocoSim(const std::string &config_file);
   ~H1MujocoSim() override;

   void Init(const std::string &config_folder) override;
   bool Step(const Eigen::VectorXd &leg_control_input, const Eigen::VectorXd &upper_control_input) override;
   void GetAllJointStateFromSensorMujoco(Eigen::VectorXd &q, Eigen::VectorXd &qdot) override;

   void SimHoldPelvis() override { mujoco_.SimHoldPelvis(); }
   void SimReleasePelvis() override { mujoco_.SimReleasePelvis(); }

   void Close() override;
   bool paused() override { return mujoco_.paused(); }
   double sim_time() override { return mujoco_.time(); }

private:
   MujocoInterface mujoco_;

   h1_sensor sensor_;

   void set_base_pos(const Eigen::Vector3d &pos) { mujoco_.set_base_pos(pos); }
   void set_base_quaternion(const Eigen::Vector4d &quat) { mujoco_.set_base_quaternion(quat); }

   const std::vector<std::string> leg_actuator_names = {
       "left_hip_yaw_joint", "left_hip_pitch_joint", "left_hip_roll_joint", "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
       "right_hip_yaw_joint", "right_hip_pitch_joint", "right_hip_roll_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"};

   const std::vector<std::string> all_joint_names_pinocchio_order = {
         "left_hip_yaw_joint",
         "left_hip_pitch_joint",
         "left_hip_roll_joint",
         "left_knee_joint",
         "left_ankle_pitch_joint",
         "left_ankle_roll_joint",
         "right_hip_yaw_joint",
         "right_hip_pitch_joint",
         "right_hip_roll_joint",
         "right_knee_joint",
         "right_ankle_pitch_joint",
         "right_ankle_roll_joint",
         "torso_joint",
         "left_shoulder_pitch_joint",
         "left_shoulder_roll_joint",
         "left_shoulder_yaw_joint",
         "left_elbow_pitch_joint",
         "left_elbow_roll_joint",
         "left_wrist_pitch_joint",
         "left_wrist_yaw_joint",
         "L_thumb_proximal_yaw_joint",
         "L_thumb_proximal_pitch_joint",
         "L_thumb_intermediate_joint",
         "L_thumb_distal_joint",
         "L_index_proximal_joint",
         "L_index_intermediate_joint",
         "L_middle_proximal_joint",
         "L_middle_intermediate_joint",
         "L_ring_proximal_joint",
         "L_ring_intermediate_joint",
         "L_pinky_proximal_joint",
         "L_pinky_intermediate_joint",
         "right_shoulder_pitch_joint",
         "right_shoulder_roll_joint",
         "right_shoulder_yaw_joint",
         "right_elbow_pitch_joint",
         "right_elbow_roll_joint",
         "right_wrist_pitch_joint",
         "right_wrist_yaw_joint",
         "R_thumb_proximal_yaw_joint",
         "R_thumb_proximal_pitch_joint",
         "R_thumb_intermediate_joint",
         "R_thumb_distal_joint",
         "R_index_proximal_joint",
         "R_index_intermediate_joint",
         "R_middle_proximal_joint",
         "R_middle_intermediate_joint",
         "R_ring_proximal_joint",
         "R_ring_intermediate_joint",
         "R_pinky_proximal_joint",
         "R_pinky_intermediate_joint"
   };

   // for joint position and velocities
   std::vector<int> all_joint_ids_pinocchio_order; // Joint IDs

   // for leg and upper body control
   std::vector<int> locomotion_actuator_ids_; // Actuator IDs
   std::vector<int> locked_actuator_ids_;     // Actuator IDs



   int step_counter = 0;
   int step_counter_threshold = 0;
};

#endif // H1_MUJOCO_SIM_HPP