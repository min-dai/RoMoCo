cmake_minimum_required(VERSION 3.0.2)
project(cassie_simulation)

## Compile as C++17, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CATKIN_DEPS
  roscpp
  roslib
  biped_core
  torque_control
  biped_control
  biped_utils
  biped_command
  biped_state_machine
)
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPS}
)

find_package(Eigen3 3.4 REQUIRED)

set(MUJOCO_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/mujoco210)
set(GLFWDIR ${CMAKE_CURRENT_LIST_DIR}/lib/mujoco210)
# Find OpenGL and GLEW libraries
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
 LIBRARIES ${PROJECT_NAME} 
 CATKIN_DEPENDS roscpp biped_core torque_control biped_control biped_utils biped_state_machine
 DEPENDS Eigen3 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${MUJOCO_DIR}/include
)


# Declare a C++ library
add_library(${PROJECT_NAME}
   src/cassie_mujoco_sim.cpp
   src/cassie_model.cpp
   src/cassie_sensor.cpp
   src/cassie_interface/cassiemujoco.c
)



# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  ${PROJECT_SOURCE_DIR}/lib/libagilitycassie.a 
                        ${MUJOCO_DIR}/bin/libmujoco210.so
                        ${MUJOCO_DIR}/bin/libglew.so
                        ${MUJOCO_DIR}/bin/libglewegl.so
                        ${MUJOCO_DIR}/bin/libglewosmesa.so
                        ${MUJOCO_DIR}/bin/libmujoco210nogl.so
                        ${MUJOCO_DIR}/bin/libglfw3.a
                        ${MUJOCO_DIR}/bin/libglfw.so.3
)



add_executable(cassie_state_machine src/cassie_main_state_machine.cpp)
target_link_libraries(cassie_state_machine
${PROJECT_NAME}
  ${catkin_LIBRARIES}
)





# Add folders to be run by python nosetests
catkin_add_nosetests(test)
catkin_add_gtest(${PROJECT_NAME}_test
  test/test.cpp

)
# target_include_directories(${PROJECT_NAME}_test PRIVATE
#   test/include
# )
target_link_libraries(${PROJECT_NAME}_test
  gtest_main
  ${PROJECT_NAME}

)
target_compile_options(${PROJECT_NAME}_test PRIVATE ${FLAGS})


