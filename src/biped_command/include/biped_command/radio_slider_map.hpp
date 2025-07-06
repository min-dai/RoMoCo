#ifndef RADIO_SLIDER_MAP_HPP
#define RADIO_SLIDER_MAP_HPP
namespace Radio
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

} // namespace Radio
#endif // RADIO_SLIDER_MAP_HPP