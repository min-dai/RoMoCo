#ifndef BIPED_CORE_BIPED_CONSTANTS_HPP
#define BIPED_CORE_BIPED_CONSTANTS_HPP

namespace romoco
{

/**
 * @brief gravity constant
 * @ingroup group_controller
 */
constexpr double grav = 9.81; // Gravity in m/s²

/**
 * @enum RobotType
 * @brief Enumeration of robot foot types, either PlaneFoot or LineFoot.
 * @ingroup group_controller
 * @details This enum defines the type of foot the robot has, which can affect its contact dynamics and control strategies.
 */
enum class RobotType
{
    PlaneFoot,
    LineFoot
};
/**
 * @enum Constants
 * @brief Enumeration of various constants used in bipedal robot control.
 * @ingroup group_controller
 * @details This enum defines constants for orbit parameters and state indices used in the control algorithms for bipedal robots.
 * The constants include:
 * - P1orbit: Control is for the sagittal plane.
 * - P2orbit: Control is for the coronal plane.
 */
enum Constants
{
    // P1 and P2 orbit for sagittal and coronal plane
    P1orbit = 1,
    P2orbit = 2,
};
/**
 * @enum FootContactStatus
 * @brief Enumeration of foot contact statuses for a bipedal robot.
 * @ingroup group_controller
 * @details This enum defines the various contact states that a robot's foot can be in, which is crucial for balance and locomotion control.
 * The statuses include:
 * - InAir: The foot is not in contact with the ground.
 * - PointContact: The foot has a point contact with the ground (e.g., planar point foot robot).
 * - ToePatchContact: The foot has a patch contact at the toe, with yaw constraints (e.g., toe/heel contact for Cassie).
 * - HeelPatchContact: The foot has a patch contact at the heel.
 * - FlatLineContact: The foot has a flat line contact with the ground (e.g., flat foot walking for Cassie).
 * - ToeLineContact: The foot has a line contact at the toe (e.g., toe/heel contact for G1).
 * - HeelLineContact: The foot has a line contact at the heel.
 * - FlatPlaneContact: The foot has a flat plane contact with the ground (e.g., flat foot walking for G1).
 */
enum class FootContactStatus
{
    InAir,
    PointContact,    // example: planar point foot robot
    ToePatchContact, // patch contact is point contact with yaw constraints, example: toe/heel contact for cassie
    HeelPatchContact,
    FlatLineContact, // example: flat foot walking for cassie
    ToeLineContact,  // example: toe/heel contact for G1
    HeelLineContact,
    FlatPlaneContact // example: flat foot walking for G1
};
/**
 * @enum AnkleMotorStatus
 * @brief Enumeration of ankle motor statuses for a bipedal robot.
 * @ingroup group_controller
 * @details This enum defines the various configurations of ankle motors, which can affect the robot's mobility and control strategies.
 * The statuses include:
 * - PassiveAll: Both pitch and roll are passive (no active control).
 * - ActivePitch: Only the pitch is actively controlled.
 * - ActiveAll: Both pitch and roll are actively controlled.
 */
enum class AnkleMotorStatus
{
    PassiveAll,  // passive pitch and roll
    ActivePitch, // active pitch
    ActiveAll    // active pitch and roll
};
/**
 * @enum StanceStatus
 * @brief Enumeration of stance statuses for a bipedal robot.
 * @ingroup group_controller
 * @details This enum defines which leg is currently in stance phase during locomotion.
 * The statuses include:
 * - LeftStance: The left leg is in the stance phase.
 * - RightStance: The right leg is in the stance phase.
 */
enum class StanceStatus
{
    LeftStance,
    RightStance
};
} // namespace romoco
#endif // BIPED_CORE_BIPED_CONSTANTS_HPP