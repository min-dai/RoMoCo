#ifndef G1_HARDWARE_INTERFACE_HPP
#define G1_HARDWARE_INTERFACE_HPP
#include <cmath>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <array>
#include <string>

#include "gamepad.hpp"

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

#include "romoco_core/interface_base.hpp"
#include "romoco_core/robot_base_pinocchio.hpp"
#include "romoco_core/contact_kf.hpp"

#include "romoco_utils/simple_timer.hpp"


namespace romoco
{

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_IMU_TORSO = "rt/secondary_imu";
static const std::string HG_STATE_TOPIC = "rt/lowstate";
template <typename T>
class DataBuffer
{
public:
    void SetData(const T &newData)
    {
        std::unique_lock<std::shared_mutex> lock(mutex);
        data = std::make_shared<T>(newData);
    }

    std::shared_ptr<const T> GetData()
    {
        std::shared_lock<std::shared_mutex> lock(mutex);
        return data;
    }

    void Clear()
    {
        std::unique_lock<std::shared_mutex> lock(mutex);
        data = nullptr;
    }

private:
    std::shared_ptr<T> data;
    std::shared_mutex mutex;
};

class G1HardwareInterface : public InterfaceBase
{
public:
    G1HardwareInterface(const std::string &network_interface, const std::string &config_file, const std::string &log_path, std::unique_ptr<romoco::robot::RobotBasePinocchio> robot);
    ~G1HardwareInterface() override = default;

    bool is_sim() const override { return false; }

    BipedProprioception ReadAndEstimate() override;

    void SendPacket() override;

    bool IsInterfaceRunning() const override { return true; }

    void DampedInitializationControl() override;

private:
    void Init(const std::string &config_folder, const std::string &log_path) override;
    void InitDDS(const std::string &network_interface);
    void GetInitialProprioception();
    void CheckSafeMotorCommands(BipedMotorCommands &commands);

    void Estimate();
    
    // // G1 joint indices
    // enum G1JointIndex {
    //     LeftHipPitch = 0,
    //     LeftHipRoll = 1,
    //     LeftHipYaw = 2,
    //     LeftKnee = 3,
    //     LeftAnklePitch = 4,
    //     LeftAnkleB = 4,
    //     LeftAnkleRoll = 5,
    //     LeftAnkleA = 5,
    //     RightHipPitch = 6,
    //     RightHipRoll = 7,
    //     RightHipYaw = 8,
    //     RightKnee = 9,
    //     RightAnklePitch = 10,
    //     RightAnkleB = 10,
    //     RightAnkleRoll = 11,
    //     RightAnkleA = 11,
    //     WaistYaw = 12,
    //     WaistRoll = 13,
    //     WaistA = 13,
    //     WaistPitch = 14,
    //     WaistB = 14,
    //     LeftShoulderPitch = 15,
    //     LeftShoulderRoll = 16,
    //     LeftShoulderYaw = 17,
    //     LeftElbow = 18,
    //     LeftWristRoll = 19,
    //     LeftWristPitch = 20,
    //     LeftWristYaw = 21,
    //     RightShoulderPitch = 22,
    //     RightShoulderRoll = 23,
    //     RightShoulderYaw = 24,
    //     RightElbow = 25,
    //     RightWristRoll = 26,
    //     RightWristPitch = 27,
    //     RightWristYaw = 28
    // };

    enum class G1Mode
    {
        PR = 0, // Series Control for Ptich/Roll Joints
        AB = 1  // Parallel Control for A/B Joints
    };

    // Data structures
    struct ImuState
    {
        std::array<float, 3> rpy = {};
        std::array<float, 3> omega = {};
        std::array<float, 3> acc = {};
        std::array<float, 4> quat = {};
    };
    // G1_NUM_MOTOR represents the total number of actuated joints in the G1 robot.
    // This value matches the joint indices defined in the commented G1JointIndex enum above (0 to 28).
        static constexpr int G1_NUM_MOTOR = 29;
    std::array<int, G1_NUM_MOTOR> g1_motor_mode;
    
    struct MotorState
    {
        std::array<float, G1_NUM_MOTOR> q = {};
        std::array<float, G1_NUM_MOTOR> dq = {};
        std::array<float, G1_NUM_MOTOR> tau_est = {};
    };

    struct MotorCommand
    {
        std::array<float, G1_NUM_MOTOR> q_target = {};
        std::array<float, G1_NUM_MOTOR> dq_target = {};
        std::array<float, G1_NUM_MOTOR> kp = {};
        std::array<float, G1_NUM_MOTOR> kd = {};
        std::array<float, G1_NUM_MOTOR> tau_ff = {};
    };

    G1Mode mode_pr_;
    uint8_t mode_machine_;

    unitree::common::Gamepad gamepad_;
    unitree::common::REMOTE_DATA_RX rx_;

    // Thread-safe data buffers
    DataBuffer<MotorState> motor_state_buffer_;
    DataBuffer<ImuState> torso_imu_buffer_;
    DataBuffer<ImuState> pelvis_imu_buffer_;
    DataBuffer<MotorCommand> motor_command_buffer_;

    // DDS communication
    unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher_;
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_;
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::IMUState_> torso_imu_subscriber_;


    // Motion switcher
    std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> motion_switcher_;

    // Callbacks
    void LowStateHandler(const void *message);
    void TorsoImuHandler(const void *message);
    void LowCommandWriter();

    void LowCommandWriterDebug();

    // Helper functions
    void InitializeJointMapping(const std::string &config_folder);
    void ConvertG1StateToProprioception();
    void ConvertMotorCommandsToG1(const BipedMotorCommands &commands);
    uint32_t Crc32Core(uint32_t *ptr, uint32_t len);

    // Raw sensor data
    std::unique_ptr<romoco::robot::RobotBasePinocchio> robot_;
    std::unique_ptr<ContactKf> contact_kf_;
    Eigen::Vector3d est_lin_vel_;
    RawSensorDataHardware sensor_hw_;
    Eigen::VectorXd torque_loco_;

    Eigen::VectorXd default_pd_q_;
    Eigen::VectorXd default_loco_q_;


    Eigen::VectorXd pd_qm0_;
    Eigen::VectorXd loco_qm0_;
    std::string interface_config_file_;

    SimpleTimer simple_timer_;


    bool initial_proprioception_received_ = false;

   // Joint position limits for motor safety checks: q_lower_bound_ and q_upper_bound_ are used to ensure commanded joint positions remain within safe operational ranges.
   Eigen::VectorXd q_lower_bound_;
   Eigen::VectorXd q_upper_bound_;
};
} // namespace romoco
#endif // G1_HARDWARE_INTERFACE_HPP
