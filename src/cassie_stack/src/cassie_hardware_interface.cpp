

#include "cassie_hardware_interface.hpp"
#include <iostream>
#include <thread>
#include <chrono>


namespace romoco
{

   CassieHardwareInterface::CassieHardwareInterface(const std::string &config_folder, const std::string &log_path, std::unique_ptr<romoco::robot::RobotBasePinocchio> robot)
       : HardwareInterfaceBase(config_folder, std::move(robot), 6)
   {
      Init(config_folder, log_path);

      // update t0
      simple_timer_.Reset();

      if (!robot_)
      {
         throw std::runtime_error("CassieHardwareInterface: Robot model is not provided.");
      }
      else
      {
         robot_->ReconfigureContactClassifier(config_folder);
      }
   }
   CassieHardwareInterface::~CassieHardwareInterface()
   {
      delete[] recvbuf;
      delete[] sendbuf;
      if (sock >= 0)
      {
         close(sock);
      }
   }


   void CassieHardwareInterface::InitUDP()
   {
      // Allocate communication buffers
      std::cout << "Using a hardware environment... Connecting to Cassie network" << std::endl;
      remote_addr_str = "10.10.10.3";
      remote_port_str = "25000";
      iface_addr_str = "10.10.10.100";
      iface_port_str = "25001";

      // Bind to network interface
      std::cout << "Bind to network at: " << (const char *)remote_addr_str.c_str() << std::endl;
      int sock = udp_init_client(remote_addr_str.c_str(), remote_port_str.c_str(), iface_addr_str.c_str(), iface_port_str.c_str());
      std::cout << "---------sock: " << sock << std::endl;

      // Prepare initial null command packet to start communication
      memset(sendbuf, 0, sendlen);
      bool received_data = false;
      printf("Running the interface node...\n");

      if (-1 == sock)
         exit(EXIT_FAILURE);

      // Create packet input/output buffers
      recvbuf = new unsigned char[recvlen];
      sendbuf = new unsigned char[sendlen];

      // Separate input/output buffers into header and payload
      header_in = recvbuf;
      data_in = &recvbuf[PACKET_HEADER_LEN];
      header_out = sendbuf;
      data_out = &sendbuf[PACKET_HEADER_LEN];

      // Prepare initial null command packet
      memset(sendbuf, 0, sendlen);
      printf("Hardware interface initialized...\n");
   }

   void CassieHardwareInterface::Init(const std::string &config_folder, const std::string &log_path)
   {
      InitLogFile(log_path);

      InitDofAndIndicesFromConfigFile(config_folder);

      // Initialize base class structures
      InitMotorCommands();


      InitProprioception();

      sensor_hw_.ResizeAll(total_motor_dof_);
   }

   void CassieHardwareInterface::Estimate()
   {
      BipedProprioception proprioception = GetBipedProprioceptionFromRawSensorDataHardware(sensor_hw_);

      robot_->UpdateKinematics(proprioception.q(loco_proprio_indices_), proprioception.qdot(loco_proprio_indices_));

      BipedEstimationKinematicsInput biped_estimation_kinematics_input = robot_->ComputeEstimationKinematicsInput();
      est_lin_vel_ = contact_kf_->Update(simple_timer_.ElapsedSinceStart(), sensor_hw_.base_lin_acc, sensor_hw_.base_ang_quat, biped_estimation_kinematics_input);

      proprioception.qdot.head(3) << est_lin_vel_;
      full_proprioception_ = proprioception;

      // Update subsets
      loco_proprioception_ = full_proprioception_.GetIndex(loco_proprio_indices_);
      pd_proprioception_ = full_proprioception_.GetIndex(pd_proprio_indices_);

      std::cout << "proprioception q: " << full_proprioception_.q(loco_proprio_indices_).transpose() << std::endl;
      std::cout << "proprioception qdot: " << full_proprioception_.qdot(loco_proprio_indices_).transpose() << std::endl;
   }

   BipedProprioception CassieHardwareInterface::ReadAndEstimate()
   {

      simple_timer_.Tick();

      // Get latest state from UDP
      // UDP Communication - Receive data
      wait_for_packet(sock, recvbuf, recvlen, nullptr, nullptr);

      // Process incoming header and write outgoing header
      process_packet_header(&header_info, header_in, header_out);

      // Unpack received data into robot output struct
      unpack_cassie_out_t(data_in, &cassie_out); // Replace with your robot's unpack function

      ConvertCassieStateToRawSensorDataHardware();

      Estimate();

      Eigen::VectorXf LogData;

      std::vector<Eigen::VectorXd> log_vectors = {est_lin_vel_, full_proprioception_.q, full_proprioception_.qdot};
      LogData = CollectLog(simple_timer_.ElapsedSinceStart(), log_vectors);
      if (logFile_.is_open())
      {
         logFile_.write(reinterpret_cast<char *>(LogData.data()), LogData.size() * sizeof(float));
      }
      return loco_proprioception_;
   }

   void CassieHardwareInterface::ConvertCassieStateToRawSensorDataHardware()
   {
      sensor_hw_.encoders_pos_pinocchio_order << cassie_out.leftLeg.hipRollDrive.position,
          cassie_out.leftLeg.hipYawDrive.position,
          cassie_out.leftLeg.hipPitchDrive.position,
          cassie_out.leftLeg.kneeDrive.position,
          cassie_out.leftLeg.tarsusJoint.position,
          cassie_out.leftLeg.footJoint.position,
          cassie_out.rightLeg.hipRollDrive.position,
          cassie_out.rightLeg.hipYawDrive.position,
          cassie_out.rightLeg.hipPitchDrive.position,
          cassie_out.rightLeg.kneeDrive.position,
          cassie_out.rightLeg.tarsusJoint.position,
          cassie_out.rightLeg.footJoint.position;
      sensor_hw_.encoders_vel_pinocchio_order << cassie_out.leftLeg.hipRollDrive.velocity,
          cassie_out.leftLeg.hipYawDrive.velocity,
          cassie_out.leftLeg.hipPitchDrive.velocity,
          cassie_out.leftLeg.kneeDrive.velocity,
          cassie_out.leftLeg.tarsusJoint.velocity,
          cassie_out.leftLeg.footJoint.velocity,
          cassie_out.rightLeg.hipRollDrive.velocity,
          cassie_out.rightLeg.hipYawDrive.velocity,
          cassie_out.rightLeg.hipPitchDrive.velocity,
          cassie_out.rightLeg.kneeDrive.velocity,
          cassie_out.rightLeg.tarsusJoint.velocity,
          cassie_out.rightLeg.footJoint.velocity;
      sensor_hw_.base_ang_quat.w() = cassie_out.pelvis.vectorNav.orientation[0];
      sensor_hw_.base_ang_quat.x() = cassie_out.pelvis.vectorNav.orientation[1];
      sensor_hw_.base_ang_quat.y() = cassie_out.pelvis.vectorNav.orientation[2];
      sensor_hw_.base_ang_quat.z() = cassie_out.pelvis.vectorNav.orientation[3];

      sensor_hw_.base_ang_vel << cassie_out.pelvis.vectorNav.angularVelocity[0],
          cassie_out.pelvis.vectorNav.angularVelocity[1],
          cassie_out.pelvis.vectorNav.angularVelocity[2];
      sensor_hw_.base_lin_acc << cassie_out.pelvis.vectorNav.linearAcceleration[0],
          cassie_out.pelvis.vectorNav.linearAcceleration[1],
          cassie_out.pelvis.vectorNav.linearAcceleration[2];
   }

   void CassieHardwareInterface::SendPacket()
   {
      std::cout << "Final motor commands: " << std::endl;
      std::cout << final_motor_commands_ << std::endl;

      // Update torque in robot model for contact classifier by compute motored joints PD control
      torque_loco_ = loco_motor_commands_.SolveFullTorque(loco_proprioception_.q(loco_proprio_motor_indices_),
                                                          loco_proprioception_.qdot(loco_proprio_motor_indices_));

      for (int i = 0; i < torque_loco_.size(); i++)
      {
         cassie_user_in.torque[i] = torque_loco_(i);
      }
      // Pack command data and send
      pack_cassie_user_in_t(&cassie_user_in, data_out); // Replace with your robot's pack function
      send_packet(sock, sendbuf, sendlen, nullptr, 0);
      robot_->set_computed_torque(torque_loco_);
   }

} // namespace romoco