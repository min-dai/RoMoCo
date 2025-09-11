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
      SB = 0,
      LV = 1,
      LH = 2,
      RV = 3,
      RH = 4,
      S1 = 5,
      S2 = 6,
      LS = 7,
      RS = 8
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