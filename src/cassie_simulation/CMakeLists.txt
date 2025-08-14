cmake_minimum_required(VERSION 3.5.0)
project(cassie_simulation)


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
find_package(mujoco_interface REQUIRED)
find_package(biped_core REQUIRED)
find_package(torque_control REQUIRED)
find_package(biped_control REQUIRED)
find_package(biped_utils REQUIRED)
find_package(biped_command REQUIRED)
find_package(biped_state_machine REQUIRED)



set(dependencies
  rclcpp
  mujoco_interface
  biped_core
  torque_control
  biped_control
  biped_utils
  biped_command
  biped_state_machine
  Eigen3
)


set(MUJOCO_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/mujoco210)
set(GLFWDIR ${CMAKE_CURRENT_LIST_DIR}/lib/mujoco210)
# Find OpenGL and GLEW libraries
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)


set(include_dirs
  ${EIGEN3_INCLUDE_DIR}
  $<BUILD_INTERFACE:${MUJOCO_DIR}/include>
  $<INSTALL_INTERFACE:include/mujoco>
)


# Declare a C++ library
add_library(${PROJECT_NAME}
   src/cassie_mujoco_sim.cpp
   src/cassie_model.cpp
   src/cassie_sensor.cpp
   src/cassie_interface/cassiemujoco.c
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${include_dirs}
)
ament_target_dependencies(${PROJECT_NAME} 
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/lib/libagilitycassie.a 
                        ${MUJOCO_DIR}/bin/libmujoco210.so
                        ${MUJOCO_DIR}/bin/libglew.so
                        ${MUJOCO_DIR}/bin/libglewegl.so
                        ${MUJOCO_DIR}/bin/libglewosmesa.so
                        ${MUJOCO_DIR}/bin/libmujoco210nogl.so
                        ${MUJOCO_DIR}/bin/libglfw3.a
                        ${MUJOCO_DIR}/bin/libglfw.so.3
)

# Install headers and library
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_export_include_directories(${include_dirs})
ament_export_libraries(${PROJECT_NAME} 
  ${PROJECT_SOURCE_DIR}/lib/libagilitycassie.a 
  ${MUJOCO_DIR}/bin/libmujoco210.so 
  ${MUJOCO_DIR}/bin/libglew.so 
  ${MUJOCO_DIR}/bin/libglewegl.so
  ${MUJOCO_DIR}/bin/libglewosmesa.so 
  ${MUJOCO_DIR}/bin/libmujoco210nogl.so
  ${MUJOCO_DIR}/bin/libglfw3.a 
  ${MUJOCO_DIR}/bin/libglfw.so.3
)

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
add_executable(cassie_state_machine src/cassie_main_state_machine.cpp)
target_link_libraries(cassie_state_machine
  ${PROJECT_NAME}
)
install(
  TARGETS cassie_state_machine
  DESTINATION lib/${PROJECT_NAME}
)

## config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
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



