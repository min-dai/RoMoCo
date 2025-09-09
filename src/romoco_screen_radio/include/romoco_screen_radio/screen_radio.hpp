#ifndef SCREEN_RADIO_HPP
#define SCREEN_RADIO_HPP

#include "romoco_types/biped_commands.hpp"
#include "romoco_screen_radio/radio_slider_map.hpp"

inline DesiredCommand getScreenCommand(Eigen::VectorXd raw_radio_data){
   DesiredCommand cmd;
   switch (static_cast<int>(raw_radio_data(Radio::SB)))
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

   cmd.values(Channel::X) = raw_radio_data(Radio::LV);
   cmd.values(Channel::Y) = raw_radio_data(Radio::LH);
   cmd.values(Channel::Z) = raw_radio_data(Radio::LS);
   cmd.values(Channel::Roll) = raw_radio_data(Radio::RS);
   cmd.values(Channel::Pitch) = raw_radio_data(Radio::RV);
   cmd.values(Channel::Yaw) = raw_radio_data(Radio::RH);
   cmd.values(Channel::StepWidth) = raw_radio_data(Radio::S2);
   cmd.values(Channel::StepTime) = raw_radio_data(Radio::S1);

   return cmd;
}


#endif // SCREEN_RADIO_HPP