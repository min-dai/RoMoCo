cmake_minimum_required(VERSION 3.5.0)
project(romoco_ros)

# Humble Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(romoco_core REQUIRED)
find_package(romoco_state_machine REQUIRED)
find_package(romoco_msgs REQUIRED)

set(dependencies 
  rclcpp
  Eigen3
  romoco_core
  romoco_state_machine
  romoco_msgs
)


add_library(${PROJECT_NAME} SHARED
    src/ros_interface_node.cpp
    src/ros_controller_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(${PROJECT_NAME} 
  ${dependencies}
)

# Install headers and library
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME} 
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)





ament_package()



