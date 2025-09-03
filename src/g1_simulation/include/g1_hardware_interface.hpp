#ifndef G1_HARDWARE_INTERFACE_HPP
#define G1_HARDWARE_INTERFACE_HPP
#include <mutex>
#include <shared_mutex>
#include <memory>
#include <array>
#include <atomic>
#include "biped_core/interface_base.hpp"


// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>


// DataBuffer template from the example
template <typename T>
class DataBuffer {
public:
    void SetData(const T &newData) {
        std::unique_lock<std::shared_mutex> lock(mutex);
        data = std::make_shared<T>(newData);
    }

    std::shared_ptr<const T> GetData() {
        std::shared_lock<std::shared_mutex> lock(mutex);
        return data ? data : nullptr;
    }

    void Clear() {
        std::unique_lock<std::shared_mutex> lock(mutex);
        data = nullptr;
    }
private:
    std::shared_ptr<T> data;
    mutable std::shared_mutex mutex;
};

class G1HardwareInterface : public InterfaceBase
{
public:
    explicit G1HardwareInterface(const std::string& network_interface);
    ~G1HardwareInterface() override;

    // Override virtual functions from InterfaceBase
    void SendPacket() override;
    
    // G1 specific methods
    bool IsConnected() const { return is_connected_.load(); }
    void SetAnkleMode(uint8_t mode) { mode_pr_ = mode; }  // 0: PR mode, 1: AB mode
    
protected:
    void Init(const std::string& config_folder) override;
    void ReadAndEstimate() override;

private:
    static constexpr int G1_NUM_MOTOR = 29;
    static constexpr double CONTROL_DT = 0.002;  // 500Hz
    
    // G1 joint indices
    enum G1JointIndex {
        LeftHipPitch = 0,
        LeftHipRoll = 1,
        LeftHipYaw = 2,
        LeftKnee = 3,
        LeftAnklePitch = 4,
        LeftAnkleB = 4,
        LeftAnkleRoll = 5,
        LeftAnkleA = 5,
        RightHipPitch = 6,
        RightHipRoll = 7,
        RightHipYaw = 8,
        RightKnee = 9,
        RightAnklePitch = 10,
        RightAnkleB = 10,
        RightAnkleRoll = 11,
        RightAnkleA = 11,
        WaistYaw = 12,
        WaistRoll = 13,
        WaistA = 13,
        WaistPitch = 14,
        WaistB = 14,
        LeftShoulderPitch = 15,
        LeftShoulderRoll = 16,
        LeftShoulderYaw = 17,
        LeftElbow = 18,
        LeftWristRoll = 19,
        LeftWristPitch = 20,
        LeftWristYaw = 21,
        RightShoulderPitch = 22,
        RightShoulderRoll = 23,
        RightShoulderYaw = 24,
        RightElbow = 25,
        RightWristRoll = 26,
        RightWristPitch = 27,
        RightWristYaw = 28
    };

    // Data structures
    struct ImuState {
        std::array<float, 3> rpy = {};
        std::array<float, 3> omega = {};
        std::array<float, 3> acc = {};
    };

    struct MotorState {
        std::array<float, G1_NUM_MOTOR> q = {};
        std::array<float, G1_NUM_MOTOR> dq = {};
        std::array<float, G1_NUM_MOTOR> tau_est = {};
    };

    struct MotorCommand {
        std::array<float, G1_NUM_MOTOR> q_target = {};
        std::array<float, G1_NUM_MOTOR> dq_target = {};
        std::array<float, G1_NUM_MOTOR> kp = {};
        std::array<float, G1_NUM_MOTOR> kd = {};
        std::array<float, G1_NUM_MOTOR> tau_ff = {};
    };

    // Thread-safe data buffers
    DataBuffer<MotorState> motor_state_buffer_;
    DataBuffer<ImuState> pelvis_imu_buffer_;
    DataBuffer<ImuState> torso_imu_buffer_;
    DataBuffer<MotorCommand> motor_command_buffer_;

    // Default PD gains
    std::array<float, G1_NUM_MOTOR> default_kp_;
    std::array<float, G1_NUM_MOTOR> default_kd_;

    // DDS communication
    ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
    ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
    ChannelSubscriberPtr<IMUState_> torso_imu_subscriber_;
    
    // Motion switcher
    std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> motion_switcher_;

    // Thread for command writer
    ThreadPtr command_writer_thread_;
    
    // State management
    std::atomic<bool> is_connected_;
    std::atomic<uint8_t> mode_pr_;  // Ankle control mode
    std::atomic<uint8_t> mode_machine_;  // G1 type (23dof/29dof etc)
    
    // Joint mapping
    std::vector<std::string> g1_joint_names_;
    std::map<int, int> g1_to_proprio_index_;  // Maps G1 index to proprioception index
    std::map<int, int> proprio_to_g1_index_;  // Maps proprioception index to G1 index
    
    // Callbacks
    void LowStateHandler(const void* message);
    void TorsoImuHandler(const void* message);
    void CommandWriterLoop();
    
    // Helper functions
    void InitializeJointMapping(const std::string& config_folder);
    void ConvertG1StateToProprioception(const MotorState& motor_state, 
                                       const ImuState& pelvis_imu,
                                       const ImuState& torso_imu);
    void ConvertMotorCommandsToG1(const BipedMotorCommands& commands);
    uint32_t ComputeCrc32(uint32_t* ptr, uint32_t len);
};

#endif // G1_HARDWARE_INTERFACE_HPP

