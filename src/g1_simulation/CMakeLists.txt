cmake_minimum_required(VERSION 3.5.0)
project(g1_simulation)

# Humble Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(mujoco_sim REQUIRED)
find_package(biped_core REQUIRED)
find_package(romoco_control REQUIRED)
find_package(romoco_output REQUIRED)
find_package(biped_utils REQUIRED)
find_package(romoco_screen_radio REQUIRED)
find_package(biped_state_machine REQUIRED)
find_package(biped_ros REQUIRED)



# Unitree SDK
# Note: This only build properly with C++ 17
include(FetchContent)

FetchContent_Declare(
    unitree_sdk
    GIT_REPOSITORY https://github.com/unitreerobotics/unitree_sdk2.git
    GIT_TAG 3a4680ae9b00df59e60f7e63cfb0fcc432a9d08d) # main # TODO: Fix the issue here

set(UNITREE_SDK_BUILD_EXAMPLES OFF CACHE BOOL "Disable building examples")
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable building examples")

FetchContent_MakeAvailable(unitree_sdk)



set(dependencies
  rclcpp
  mujoco_sim
  biped_core
  romoco_control
  romoco_output
  biped_utils
  romoco_screen_radio
  biped_state_machine
  Eigen3
  biped_ros
)

# Library target
add_library(${PROJECT_NAME}
   src/g1_model_leg.cpp
   src/g1_model.cpp
   src/g1_mujoco_interface.cpp
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



## Ros Node
add_executable(g1_state_machine src/g1_main_state_machine.cpp)
target_link_libraries(g1_state_machine
  ${PROJECT_NAME}
)



## Ros Node
add_executable(g1_hardware_interface_main 
  src/g1_hardware_interface_main.cpp
  src/g1_hardware_interface.cpp
)
target_link_libraries(g1_hardware_interface_main
  ${PROJECT_NAME}
  unitree_sdk2
)



## Ros Node
add_executable(g1_mujoco_interface_node_main 
  src/g1_mujoco_interface_node_main.cpp
)
target_link_libraries(g1_mujoco_interface_node_main
  ${PROJECT_NAME}
)

add_executable(g1_dds
  src/g1_ankle_swing_dds_example.cpp
)
target_link_libraries(g1_dds
  ${PROJECT_NAME}
  unitree_sdk2
)


add_executable(g1_controller_node_main src/g1_controller_node_main.cpp)
target_link_libraries(g1_controller_node_main
  ${PROJECT_NAME}
)





# add_executable(g1_momentum src/g1_state_machine_momentum.cpp)
# target_link_libraries(g1_momentum
#   ${PROJECT_NAME}
# )

install(
  TARGETS g1_state_machine  g1_mujoco_interface_node_main g1_controller_node_main g1_hardware_interface_main g1_dds
  DESTINATION lib/${PROJECT_NAME}
)

# install(
#   TARGETS g1_hardware_interface
#   DESTINATION lib/${PROJECT_NAME}
# )

## config files
install(DIRECTORY config_18dof/
  DESTINATION share/${PROJECT_NAME}/config_18dof
)
install(DIRECTORY config_18dof_hardware/
  DESTINATION share/${PROJECT_NAME}/config_18dof_hardware
)
install(DIRECTORY config_29dof/
  DESTINATION share/${PROJECT_NAME}/config_29dof
)
install(DIRECTORY model_files/
  DESTINATION share/${PROJECT_NAME}/model_files
)
install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME})

# # Tests
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(${PROJECT_NAME}_test 
#     test/test_slider_gui.cpp
#   )
#   target_include_directories(${PROJECT_NAME}_test PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}/include
#   )
#   target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

#   # find_package(ament_lint_auto REQUIRED)
#   # ament_lint_auto_find_test_dependencies()
# endif()


ament_package()
