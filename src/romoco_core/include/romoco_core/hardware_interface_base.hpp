#ifndef HARDWARE_INTERFACE_BASE_HPP
#define HARDWARE_INTERFACE_BASE_HPP

#include <Eigen/Dense>
#include "romoco_core/interface_base.hpp"

#include "romoco_core/biped_sensors.hpp"
#include "romoco_core/robot_base_pinocchio.hpp"
#include "romoco_core/contact_kf.hpp"

#include "romoco_utils/simple_timer.hpp"
namespace romoco
{
   /**
    * @class HardwareInterfaceBase
    * @brief An abstract base class for hardware interfaces, providing a common interface for reading sensors and writing commands.
    * @ingroup group_interface
    * This class defines the essential methods that any hardware interface must implement to interact with physical hardware.
    * It includes methods for reading sensor data and writing control commands, ensuring a consistent interface across
    */
   class HardwareInterfaceBase : public InterfaceBase
   {
   public:
      HardwareInterfaceBase(
          const std::string &config_folder,
          std::unique_ptr<romoco::robot::RobotBasePinocchio> robot,
          int dof_encoder_per_leg)
          : InterfaceBase(false),
            robot_(std::move(robot)),
            contact_kf_(std::make_unique<ContactKf>(config_folder, dof_encoder_per_leg)) {}; // Hardware interface is not ready when initialized

      virtual ~HardwareInterfaceBase() = default;

      bool is_sim() const override { return false; }
      bool IsInterfaceRunning() const override { return true; }

   protected:
      // Raw sensor data
      std::unique_ptr<romoco::robot::RobotBasePinocchio> robot_;
      std::unique_ptr<ContactKf> contact_kf_;
      Eigen::Vector3d est_lin_vel_;
      RawSensorDataHardware sensor_hw_;

      SimpleTimer simple_timer_;


      Eigen::VectorXd torque_loco_;
   };
} // namespace romoco

#endif // HARDWARE_INTERFACE_BASE_HPP