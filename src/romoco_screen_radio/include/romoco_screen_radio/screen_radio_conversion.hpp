#ifndef SCREEN_RADIO_HPP
#define SCREEN_RADIO_HPP

#include "romoco_types/biped_commands.hpp"
#include "romoco_screen_radio/radio_slider_map.hpp"



/**
 * @brief Converts raw radio data from the screen radio into a DesiredCommand structure.
 *
 * This function interprets the mode from the SB channel of the input vector and maps it to the corresponding
 * Mode enum value. It then assigns the remaining channels from the input vector to the appropriate fields
 * in the DesiredCommand's values array.
 *
 * @param raw_radio_data An Eigen::VectorXd containing raw radio data, where each index corresponds to a specific channel defined in ScreenRadio.
 * @return DesiredCommand The command structure populated with mode and channel values extracted from the input data.
 */
inline DesiredCommand ConvertScreenRadioToDesiredCommand(Eigen::VectorXd raw_radio_data){
   DesiredCommand cmd;
   switch (static_cast<int>(raw_radio_data(ScreenRadio::SB)))
   {
   case -1:
      cmd.mode = Mode::Null;
      break;
   case -2:
      cmd.mode = Mode::InAir;
      break;
   case 0:
      cmd.mode = Mode::Standing;
      break;
   case 1:
      cmd.mode = Mode::Walking;
      break;
   }

   cmd.values(Channel::X) = raw_radio_data(ScreenRadio::LV);
   cmd.values(Channel::Y) = raw_radio_data(ScreenRadio::LH);
   cmd.values(Channel::Z) = raw_radio_data(ScreenRadio::LS);
   cmd.values(Channel::Roll) = raw_radio_data(ScreenRadio::RS);
   cmd.values(Channel::Pitch) = raw_radio_data(ScreenRadio::RV);
   cmd.values(Channel::Yaw) = raw_radio_data(ScreenRadio::RH);
   cmd.values(Channel::StepWidth) = raw_radio_data(ScreenRadio::S2);
   cmd.values(Channel::StepTime) = raw_radio_data(ScreenRadio::S1);

   return cmd;
}


#endif // SCREEN_RADIO_HPP