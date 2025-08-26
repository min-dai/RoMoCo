#ifndef BIPED_COMMANDS_HPP
#define BIPED_COMMANDS_HPP

#include <Eigen/Dense>

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

enum class Mode
{
   Null = -1,
   InAir = -2,
   Standing = 0,
   Walking = 1,
};

struct DesiredCommand
{
   Mode mode;                                                               // see enum class Mode for possible values
   Eigen::Matrix<double, static_cast<int>(Channel::NumChannels), 1> values; // see Channel for indices, values are bewteen -1 and 1

   DesiredCommand() : mode(Mode::Null)
   {
      values.setZero();
   }
};

#endif // BIPED_COMMANDS_HPP