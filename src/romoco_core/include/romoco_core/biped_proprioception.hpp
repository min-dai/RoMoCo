#ifndef BIPED_PROPRIOCEPTION_HPP
#define BIPED_PROPRIOCEPTION_HPP

#include <Eigen/Dense>

#include "romoco_core/biped_sensors.hpp"
namespace romoco
{

/** 
 * @struct BipedProprioception
 * @brief Represents the proprioceptive state of a bipedal robot, including joint positions and velocities.
 * @ingroup group_controller
 */
struct BipedProprioception
{
    /**
     * @brief Joint positions vector, including floating base and encoders, ordered as in Pinocchio.
     */
   Eigen::VectorXd q;   
    /**
     * @brief Joint velocities vector, including floating base and encoders, ordered as in Pinocchio.
     */
   Eigen::VectorXd qdot; 
   friend std::ostream &operator<<(std::ostream &os, const BipedProprioception &data)
   {
       os << "Proprioception q: " << data.q.transpose() << "\n";
       os << "Proprioception qdot: " << data.qdot.transpose() << "\n";
       return os;
   }
   /**
    * @brief Extracts a subset of the proprioceptive data based on provided indices.
    * @param indices A vector of indices specifying which elements to extract.
    * @return A new BipedProprioception instance containing only the selected elements.
    */
   BipedProprioception GetIndex(const std::vector<int> &indices) const
   {
       BipedProprioception result;
       result.q = q(indices);
       result.qdot = qdot(indices);
       return result;
   }
   /**
    * @brief Resizes the joint position and velocity vectors to the specified degree of freedom (dof).
    * @param dof The new size for the joint vectors.
    */
   void ResizeAll(int dof)
   {
       q.resize(dof);
       qdot.resize(dof);
   }
    /**
     * @brief Sets all joint positions and velocities to zero for the specified degree of freedom (dof).
     * @param dof The size of the joint vectors to be set to zero.
     */
   void ZeroAll(int dof)
   {
       q = Eigen::VectorXd::Zero(dof);
       qdot = Eigen::VectorXd::Zero(dof);
   }
};
/**
 * @brief Converts sensor data post-estimation to biped proprioception.
 * @ingroup group_controller
 * @param sensor_data The sensor data after estimation.
 * @return A BipedProprioception instance containing joint positions and velocities.
 */
BipedProprioception GetBipedProprioceptionFromSensorDataPostEstimation(const SensorDataPostEstimation &sensor_data);
/**
 * @brief Converts raw hardware sensor data to biped proprioception, using provided estimation.
 * @ingroup group_controller
 * @param raw_sensor_data The raw sensor data from hardware.
 * @param estimation The biped state estimation.
 * @return A BipedProprioception instance containing joint positions and velocities.
 */
BipedProprioception GetBipedProprioceptionFromRawSensorDataHardware(const RawSensorDataHardware &raw_sensor_data, const BipedEstimation &estimation);
/**
 * @brief Converts raw hardware sensor data to biped proprioception, assuming zero estimation.
 * @ingroup group_controller
 * @param raw_sensor_data The raw sensor data from hardware.
 * @return A BipedProprioception instance containing joint positions and velocities.
 * @note This function assumes that the estimation is zero.
 */
BipedProprioception GetBipedProprioceptionFromRawSensorDataHardware(const RawSensorDataHardware &raw_sensor_data);

} // namespace romoco

#endif // BIPED_PROPRIOCEPTION_HPP