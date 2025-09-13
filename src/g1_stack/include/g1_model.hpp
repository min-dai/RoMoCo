#ifndef G1_MODEL_HPP
#define G1_MODEL_HPP

#include "g1_model_leg.hpp"

namespace romoco
{
    namespace robot
    {
/**
 * @brief Pinocchio model for the full Unitree G1 robot with 29 joints.
 * @ingroup group_g1_examples
 * @details This class extends the G1ModelLeg to include the upper body joints of the Unitree G1 robot.
 * This class is used for hierachical QP control for natural arm swing motion.
 */
        class G1Model : public G1ModelLeg
{
public:
    enum JointIndexFull
    {
        WaistYaw = 18,
        WaistRoll = 19,
        WaistPitch = 20,
        LeftShoulderPitch,
        LeftShoulderRoll,
        LeftShoulderYaw,
        LeftElbowPitch,
        RightShoulderPitch,
        RightShoulderRoll,
        RightShoulderYaw,
        RightElbowPitch,
    };

    G1Model(const std::string &urdf_path, const std::vector<std::string> &locked_encoder_names, const Eigen::VectorXd &locked_joints_q);
    G1Model(const std::string &config_folder);
    ~G1Model() override = default;
    

    std::vector<int> actuated_q_idx(AnkleMotorStatus left_ankle_status, AnkleMotorStatus right_ankle_status) const override;
    std::vector<int> actuated_u_idx(AnkleMotorStatus left_ankle_status, AnkleMotorStatus right_ankle_status) const override;


    void AddFrames() override;

    void InitActuation() override;

    void InitJointKinematics() override;


private:

};
    } // namespace robot
} // namespace romoco

#endif // G1_MODEL_HPP