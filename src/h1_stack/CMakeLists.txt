cmake_minimum_required(VERSION 3.5.0)
project(h1_stack)

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
find_package(romoco_mujoco REQUIRED)
find_package(romoco_core REQUIRED)
find_package(romoco_control REQUIRED)
find_package(romoco_output REQUIRED)
find_package(romoco_utils REQUIRED)
find_package(romoco_screen_radio REQUIRED)
find_package(romoco_state_machine REQUIRED)
find_package(romoco_ros REQUIRED)


set(dependencies
  rclcpp
  romoco_mujoco
  romoco_core
  romoco_control
  romoco_output
  romoco_utils
  romoco_screen_radio
  romoco_state_machine
  Eigen3
  romoco_ros
)

# Library target
add_library(${PROJECT_NAME}
   src/h1_model_leg.cpp
   src/h1_mujoco_interface.cpp
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
add_executable(h1_state_machine src/h1_main_state_machine.cpp)
target_link_libraries(h1_state_machine
  ${PROJECT_NAME}
)






## Ros Node
add_executable(h1_mujoco_interface_node_main 
  src/h1_mujoco_interface_node_main.cpp
)
target_link_libraries(h1_mujoco_interface_node_main
  ${PROJECT_NAME}
)


add_executable(h1_controller_node_main src/h1_controller_node_main.cpp)
target_link_libraries(h1_controller_node_main
  ${PROJECT_NAME}
)



install(
  TARGETS h1_state_machine  h1_mujoco_interface_node_main h1_controller_node_main 
  DESTINATION lib/${PROJECT_NAME}
)

# install(
#   TARGETS h1_hardware_interface
#   DESTINATION lib/${PROJECT_NAME}
# )

## config files
install(DIRECTORY config_18dof/
  DESTINATION share/${PROJECT_NAME}/config_18dof
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
