#ifndef BIPED_CORE_BIPED_CONSTANTS_HPP
#define BIPED_CORE_BIPED_CONSTANTS_HPP

constexpr double grav = 9.81; // Gravity in m/s²

enum class RobotType
{
    PlaneFoot,
    LineFoot
};

enum Constants
{
    // P1 and P2 orbit for sagittal and coronal plane
    P1orbit = 1,
    P2orbit = 2,

    // states
    BasePosX = 0,
    BasePosY = 1,
    BasePosZ = 2,
    BaseRotZ = 3,
    BaseRotY = 4,
    BaseRotX = 5,
};

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

enum class AnkleMotorStatus
{
    PassiveAll,  // passive pitch and roll
    ActivePitch, // active pitch
    ActiveAll    // active pitch and roll
};

enum class StanceStatus
{
    LeftStance,
    RightStance
};

#endif // BIPED_CORE_BIPED_CONSTANTS_HPP