#ifndef RADIO_SLIDER_MAP_HPP
#define RADIO_SLIDER_MAP_HPP

namespace romoco
{
/**
 * @namespace romoco::ScreenRadio
 * @brief Namespace for Screen Radio related enumerations and functionalities.
 * @ingroup group_ui
 * @details This namespace contains enumerations for mapping radio slider controls and commands.
 * It is used in the context of GUI applications for controlling radio sliders.
 */
   namespace ScreenRadio
{
   enum RadioSliderMap
   {
      Mode = 0,
      X = 1,
      Y = 2,
      Z = 3,
      Roll = 4,
      Pitch = 5,
      Yaw = 6,
      StepTime = 7,
      StepWidth = 8
   };

   enum RadioCommand
   {
      Null = -1,
      InAir = -2,
      Standing = 0,
      Walking = 1,
   };

} // namespace ScreenRadio
} // namespace romoco
#endif // RADIO_SLIDER_MAP_HPP