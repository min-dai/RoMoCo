#ifndef PREP_PROPRIOCEPTION_HPP
#define PREP_PROPRIOCEPTION_HPP

#include <romoco_core/biped_proprioception.hpp>

namespace romoco
{

BipedProprioception GetBipedProprioceptionFromSensorDataPostEstimation(const SensorDataPostEstimation &sensor_data);

BipedProprioception GetBipedProprioceptionFromRawSensorDataHardware(const RawSensorDataHardware &raw_sensor_data, const BipedEstimation &estimation);
//assuming estimation is zero
BipedProprioception GetBipedProprioceptionFromRawSensorDataHardware(const RawSensorDataHardware &raw_sensor_data);
} // namespace romoco

#endif // PREP_PROPRIOCEPTION_HPP