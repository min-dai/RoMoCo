#include <gtest/gtest.h>
#include "mujoco_interface/mujoco_interface.hpp"
#include <iostream>
class MujocoInterfaceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Path to a small test MJCF/XML
    std::string model_path = std::string(__FILE__);
    auto dir = model_path.substr(0, model_path.find_last_of("/\\"));
    model_path = dir + "/test_model.xml";
    std::cout << "Model path: " << model_path << std::endl;
    ASSERT_TRUE(mj_interface.Init(model_path.c_str(), 640, 480))
        << "Failed to load model at: " << model_path;
  }

  void TearDown() override {
    mj_interface.Close();
  }

  MujocoInterface mj_interface;
};

// TEST_F(MujocoInterfaceTest, GetActuatorIdsByName) {
//   std::vector<std::string> names = {"actuator_1", "actuator_2"};
//   std::vector<int> ids = mj_interface.GetActuatorIdsByName(names);
//   EXPECT_EQ(ids.size(), 2);
//   EXPECT_GE(ids[0], 0);
//   EXPECT_GE(ids[1], 0);
// }

// TEST_F(MujocoInterfaceTest, GetJointIdsByName) {
//   std::vector<std::string> names = {"joint_1", "joint_2"};
//   auto ids = mj_interface.GetJointIdsByName(names);
//   EXPECT_EQ(ids.size(), 2);
//   EXPECT_GE(ids[0], 0);
//   EXPECT_GE(ids[1], 0);
// }
