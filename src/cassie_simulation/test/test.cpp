#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <iostream>

#include "biped_control/walking_output_fp.hpp"
#include "biped_control/standing_output.hpp"

#include "cassie_mujoco.hpp"
#include "cassie_model.hpp"
#include "cassie_sensor.hpp"

#include "torque_control/torque_solver_tscqp.hpp"
#include "torque_control/pd_controller.hpp"
#include "torque_control/torque_solver_inv_dyn.hpp"
#include "torque_control/torque_solver_vel_ik.hpp"
#include "biped_utils/yaml_parser.hpp"

using namespace Eigen;
using namespace std;
class CassieTest : public ::testing::Test
{
protected:
   // Example states for testing (you can extend this)
   std::vector<Eigen::VectorXd> test_q;
   std::vector<Eigen::VectorXd> test_dq;
   std::vector<int> test_t;

   // void SetUp() override
   // {
   //    // Example: Initialize test cases with random or predefined states
   //    int dim = 10; // Assuming 10 DOF robot
   //    for (int i = 0; i < 5; ++i)
   //    {
   //       Eigen::VectorXd q = Eigen::VectorXd::Random(dim);
   //       Eigen::VectorXd dq = Eigen::VectorXd::Random(dim);
   //       test_q.push_back(q);
   //       test_dq.push_back(dq);
   //    }
   // }

   // Comparison function for Eigen vectors
   bool EigenApproxEqual(const Eigen::VectorXd &a, const Eigen::VectorXd &b, double tolerance = 1e-6)
   {
      return a.isApprox(b, tolerance);
   }
};

TEST_F(CassieTest, TestStandingController)
{
   std::string home = std::string(getenv("HOME"));
   std::string config_file = home + "/biped_simulation/src/cassie_simulation/config/standing_config.yaml";

   std::string mujoco_config_file = home + "/biped_simulation/src/cassie_simulation/config/mujoco_config.yaml";

   YAMLParser yaml_parser(mujoco_config_file);
   std::string urdf_name = yaml_parser.get_string("urdf_name");
   std::string urdf_path = home + "/biped_simulation/src/cassie_simulation/cassie_model_files/" + urdf_name;
   std::vector<std::string> locked_joints_names = yaml_parser.get_string_vector("locked_joints_names");

   std::shared_ptr<CassieModel> robot_ptr = std::make_shared<CassieModel>(urdf_path, locked_joints_names);

   std::shared_ptr<StandingOutput> output = std::make_shared<StandingOutput>(config_file, robot_ptr);
   std::shared_ptr<TorqueSolverTSCQP> torque_solver_qp = std::make_shared<TorqueSolverTSCQP>(config_file, robot_ptr, output);
   std::unique_ptr<TorqueSolverInvDyn> torque_solver_inv_dyn = std::make_unique<TorqueSolverInvDyn>(config_file, robot_ptr, output);

   cassie_sensor sensor;

   // Initialize Mujoco simulator
   CassieMujoco mujocosim;
   mujocosim.Init(mujoco_config_file, sensor);

   VectorXd fake_radio = VectorXd::Zero(10);

   VectorXd pseudo_leg_input = VectorXd::Zero(10);
   VectorXd q = VectorXd::Zero(robot_ptr->nq());
   VectorXd dq = VectorXd::Zero(robot_ptr->nv());

   // while (mujocosim.getTime() < 0.001)
   // {

   //    mujocosim.step(pseudo_leg_input, sensor);

   //    if (!mujocosim.paused())
   //    {
         getAllJointStateFromSensorMujoco(sensor, q, dq);

         q << -0.000487539, -5.80898e-07, 0.998611, -6.27777e-06, -0.00563945, -5.77493e-07,
             0.0045099, 0, 0.502043, -1.2233, 1.45405, -1.60915,
             -0.0045099, 0, 0.502043, -1.2233, 1.45405, -1.60915;
         dq.setConstant(0);

         robot_ptr->UpdateAll(q, dq);
         // t_old not used for standing controller
         output->UpdateOutput(fake_radio, mujocosim.getTime(), 0);

         VectorXd u_qp = torque_solver_qp->Solve();
         VectorXd u_qp_noineq = torque_solver_qp->Solve_noIneq();
         VectorXd u_inv_dyn = torque_solver_inv_dyn->Solve();

         pseudo_leg_input = u_inv_dyn;

         // Compare outputs
         EXPECT_TRUE(EigenApproxEqual(u_qp, u_qp_noineq));
   //    }
   // }
}