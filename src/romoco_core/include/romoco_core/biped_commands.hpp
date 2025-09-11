#ifndef BIPED_COMMANDS_HPP
#define BIPED_COMMANDS_HPP

#include <Eigen/Dense>

namespace romoco
{

   /**
    * @namespace Channel
    * @brief Enumeration of control channels for desired robot commands.\
    * @ingroup group_controller
    * @details This namespace defines the indices for various control channels used in commanding a bipedal robot.
    * The channels include translational and rotational movements, as well as step parameters.
    */
namespace Channel
{
   enum DesiredChannel
   {
      X = 0,
      Y = 1,
      Z = 2,
      Roll = 3,
      Pitch = 4,
      Yaw = 5,
      StepWidth = 6,
      StepTime = 7,
      NumChannels = 8
   };

}
/**  
 * @enum Mode
 * @brief Enumeration of operational modes for the bipedal robot.
 * @ingroup group_controller
 * @details This enum defines the various modes the robot can operate in, including:
 * - Null: No specific mode is set.
 * - InAir: The robot is hanging in air by its torso.
 * - Standing: The robot is in a standing position.
 * - Walking: The robot is in motion, walking.
 */
enum class Mode
{
   Null = -1,
   InAir = -2,
   Standing = 0,
   Walking = 1,
};
/**
 * @struct DesiredCommand
 * @brief Structure representing the desired command for a bipedal robot.
 * @ingroup group_controller
 * @details This structure contains the operational mode and a vector of channel values.
 * The mode indicates the robot's current state, while the values vector holds the desired
 * inputs for various control channels, each ranging between -1 and 1.
 */
struct DesiredCommand
{
   Mode mode;                                                               // see enum class Mode for possible values
   Eigen::Matrix<double, static_cast<int>(Channel::NumChannels), 1> values; // see Channel for indices, values are bewteen -1 and 1

   DesiredCommand() : mode(Mode::Null)
   {
      values.setZero();
   }
   //overload <<
   friend std::ostream &operator<<(std::ostream &os, const DesiredCommand &cmd)
   {
      switch (cmd.mode)
      {
      case Mode::Standing:
         os << "Mode: Standing, ";
         break;
      case Mode::Walking:
         os << "Mode: Walking, ";
         break;
      case Mode::InAir:
         os << "Mode: InAir, ";
         break;
      case Mode::Null:
         os << "Mode: Null, ";
         break;
      default:
         os << "Mode: Unknown, ";
         break;
      }
      os << "Values: " << cmd.values.transpose();
      return os;
   }
};

} // namespace romoco

#endif // BIPED_COMMANDS_HPP