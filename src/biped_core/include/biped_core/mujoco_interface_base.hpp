#ifndef MUJOCO_INTERFACE_BASE_HPP
#define MUJOCO_INTERFACE_BASE_HPP
#include "biped_core/interface_base.hpp"
#include "biped_utils/yaml_parser.hpp"
#include "biped_types/biped_proprioception.hpp"

class MujocoInterfaceBase : public InterfaceBase
{
public:
   MujocoInterfaceBase() = default;
   virtual ~MujocoInterfaceBase() = default;

   bool IsInterfaceRunning()  override{
      return !paused();
   }

   void ReadAndEstimate() override;
   void SendPacket() override;


   void GetAllJointStateFromSensorMujoco(Eigen::VectorXd &q, Eigen::VectorXd &qdot);


   virtual bool Step(const Eigen::VectorXd &leg_control_input, const Eigen::VectorXd &upper_control_input) = 0;
   
   virtual void SimHoldPelvis() = 0;
   virtual void SimReleasePelvis() = 0;

   virtual bool paused() = 0;
   virtual double sim_time() = 0;

protected:
   struct VideoSetting
   {
      bool record_video = false;
      int video_width = 1000;
      int video_height = 1000;
      int video_fps = 30;
      std::string video_name = "video.mp4";
      void InitVideoSetting(const std::string &config_file)
      {
         YAMLParser yaml_parser(config_file);
         record_video = yaml_parser.get_bool("video_settings/record_video");
         video_width = yaml_parser.get_int("video_settings/video_width");
         video_height = yaml_parser.get_int("video_settings/video_height");
         video_fps = yaml_parser.get_int("video_settings/video_fps");
         video_name = yaml_parser.get_string("video_settings/video_name");
      }

   } videoSetting;

   SensorDataPostEstimation sensor_;
   RawSensorDataHardware sensor_hw_;

   Eigen::VectorXd torque_loco_;
   Eigen::VectorXd torque_pd_;

   int render_loop_counter_ = 0;
   int render_loop_counter_threshold_ = 40;

};
#endif // MUJOCO_INTERFACE_BASE_HPP