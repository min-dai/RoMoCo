#ifndef CASSIE_MUJOCO_INTERFACE_HPP
#define CASSIE_MUJOCO_INTERFACE_HPP

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "romoco_core/hardware_interface_base.hpp"
#include "romoco_core/contact_kf.hpp"
#include <cassie_interface/cassie_out_t.h>
#include <cassie_interface/cassie_user_in_t.h>
#include <cassie_interface/cassiemujoco.h> // mujoco
#include <cassie_interface/udp.h>
namespace romoco
{
    /**
     * @class CassieHardwareInterface
     * @brief A Hardware interface specialized for the Cassie robot.
     * @ingroup group_cassie_examples
     */
    class CassieHardwareInterface : public HardwareInterfaceBase
    {
    public:
        CassieHardwareInterface();
        CassieHardwareInterface(const std::string &config_file, const std::string &log_path);
        CassieHardwareInterface(const std::string &config_file, const std::string &log_path, std::unique_ptr<romoco::robot::RobotBasePinocchio> robot);
        ~CassieHardwareInterface() override;

        BipedProprioception ReadAndEstimate() override;
        void SendPacket() override;

    private:
        void Init(const std::string &config_file, const std::string &log_path) override;
        void InitUDP();
        void Close();

        void ConvertCassieStateToRawSensorDataHardware();

        void Estimate();

        // Network communication
        std::string remote_addr_str, remote_port_str, iface_addr_str, iface_port_str;
        int sock;

        // Communication buffers
        static const int dinlen = CASSIE_OUT_T_PACKED_LEN; // Replace with your robot's packet length
        static const int doutlen = CASSIE_USER_IN_T_PACKED_LEN;
        static const int recvlen = PACKET_HEADER_LEN + dinlen;
        static const int sendlen = PACKET_HEADER_LEN + doutlen;
        unsigned char *recvbuf;
        unsigned char *sendbuf;

        // Packet components
        const unsigned char *header_in;
        const unsigned char *data_in;
        unsigned char *header_out;
        unsigned char *data_out;

        // Header info
        packet_header_info_t header_info = {0};

        cassie_out_t cassie_out;
        cassie_user_in_t cassie_user_in = {0};


    };
} // namespace romoco
#endif // CASSIE_MUJOCO_INTERFACE_HPP