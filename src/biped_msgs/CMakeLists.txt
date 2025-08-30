cmake_minimum_required(VERSION 3.5.0)
project(biped_msgs)

# Humble Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(Eigen3 3.4 REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BipedMotorCommands.msg"
  "msg/BipedProprioception.msg"
)
ament_export_dependencies(rosidl_default_runtime)




ament_package()
