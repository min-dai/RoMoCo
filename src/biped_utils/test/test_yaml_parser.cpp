#include <gtest/gtest.h>
#include "biped_utils/yaml_parser.hpp"
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;

class YAMLParserTest : public ::testing::Test {
protected:
  std::string test_file;

  void SetUp() override {
    test_file = "test_config.yaml";
    std::ofstream ofs(test_file);
    ofs << R"(
int_value: 42
double_value: 3.14
bool_value: true
string_value: "hello"
vector_value: [1.0, 2.0, 3.0]
string_list: ["a", "b", "c"]
nested:
  level1:
    level2: "deep"
)";
    ofs.close();
  }

  void TearDown() override {
    fs::remove(test_file);
  }
};

TEST_F(YAMLParserTest, LoadAndRetrieveValues) {
  YAMLParser parser(test_file);

  EXPECT_EQ(parser.get_int("int_value"), 42);
  EXPECT_DOUBLE_EQ(parser.get_double("double_value"), 3.14);
  EXPECT_TRUE(parser.get_bool("bool_value"));
  EXPECT_EQ(parser.get_string("string_value"), "hello");

  Eigen::VectorXd vec = parser.get_VectorXd("vector_value");
  ASSERT_EQ(vec.size(), 3);
  EXPECT_DOUBLE_EQ(vec[0], 1.0);
  EXPECT_DOUBLE_EQ(vec[1], 2.0);
  EXPECT_DOUBLE_EQ(vec[2], 3.0);

  std::vector<std::string> str_vec = parser.get_string_vector("string_list");
  ASSERT_EQ(str_vec.size(), 3);
  EXPECT_EQ(str_vec[0], "a");
  EXPECT_EQ(str_vec[1], "b");
  EXPECT_EQ(str_vec[2], "c");

  EXPECT_EQ(parser.get_string("nested/level1/level2"), "deep");
}

TEST_F(YAMLParserTest, UninitializedError) {
  YAMLParser parser;
  EXPECT_THROW(parser.get_int("int_value"), std::runtime_error);
}

TEST_F(YAMLParserTest, MissingKeyError) {
  YAMLParser parser(test_file);
  EXPECT_THROW(parser.get_int("does/not/exist"), std::runtime_error);
}
