cmake_minimum_required(VERSION 3.5.0)
project(mujoco_interface)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)



find_package(Eigen3 3.4 REQUIRED)
#print out the Eigen3 include directory
message( "Eigen3 include directory: ${EIGEN3_INCLUDE_DIR}")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

SET(MUJOCO_DIR $ENV{HOME}/repos/mujoco-3.2.6)
set(GLFW_INCLUDE_DIR /usr/include)
set(GLFW_LIBRARY /usr/lib/x86_64-linux-gnu/libglfw.so)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include ${MUJOCO_DIR}/include ${GLFW_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR}
 LIBRARIES ${PROJECT_NAME} ${MUJOCO_DIR}/lib/libmujoco.so ${GLFW_LIBRARY}
 CATKIN_DEPENDS roscpp
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
  ${MUJOCO_DIR}/include
  ${GLFW_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
   src/mujoco_interface.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${MUJOCO_DIR}/lib/libmujoco.so
  ${GLFW_LIBRARY}
  Eigen3::Eigen
)


## Test
catkin_add_gtest(${PROJECT_NAME}_test
  test/test_mujoco_interface.cpp
)
target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}
  gtest_main
)
target_compile_options(${PROJECT_NAME}_test PRIVATE ${FLAGS}) 