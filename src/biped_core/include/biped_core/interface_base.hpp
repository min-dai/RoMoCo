#ifndef INTERFACE_BASE_HPP
#define INTERFACE_BASE_HPP

#include <Eigen/Dense>
#include "biped_core/robot_base_pinocchio.hpp"
#include "biped_types/biped_proprioception.hpp"
#include "biped_types/biped_motor_commands.hpp"
class InterfaceBase
{
public:
   explicit InterfaceBase(std::shared_ptr<RobotBasePinocchio> robot)
       : robot_(std::move(robot)) {}

   virtual ~InterfaceBase() = default;

   // Pull raw sensors and update internal estimation.
  virtual void ReadAndEstimate() = 0;

  // Consume locomotion controller outputs; update internal state.
  virtual void AcceptControllerCommand(const BipedMotorCommands& ctrl_cmd) = 0;

  // Synthesize final motor commands to send to actuators/transport.
  // This include using PD controller to fixed joints that's not used by the locomotion controller.
  virtual BipedMotorCommands ComposeActuatorCommand() const = 0;

protected:

   std::vector<int> GetJointIndicesFromSubset(
       const std::vector<std::string>& all_joint_names_pinocchio_order,
       const std::vector<std::string>& subset_joint_names);




   std::shared_ptr<RobotBasePinocchio> robot_;
};

#endif // INTERFACE_BASE_HPP