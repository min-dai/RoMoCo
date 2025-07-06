cmake_minimum_required(VERSION 3.0.2)
project(h1_simulation)

## Compile as C++17, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CATKIN_DEPS
  roscpp
  roslib
  mujoco_interface
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

find_package(pinocchio REQUIRED)

find_package(Qt5Widgets REQUIRED)

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
 CATKIN_DEPENDS roscpp mujoco_interface biped_core torque_control biped_control biped_utils biped_state_machine
 DEPENDS Eigen3 pinocchio
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
  ${PINOCCHIO_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)


# Declare a C++ library
add_library(${PROJECT_NAME}
   src/h1_model_leg.cpp
   src/h1_sensor.cpp
   src/h1_mujoco_sim.cpp
)



# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  pinocchio::pinocchio
)







add_executable(h1_state_machine src/h1_main_state_machine.cpp)
target_link_libraries(h1_state_machine
${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


# add_executable(test_node src/test_main.cpp)

# target_link_libraries(test_node
# ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mujoco_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# Add folders to be run by python nosetests
# catkin_add_nosetests(test)
# catkin_add_gtest(${PROJECT_NAME}_test
#   test/test.cpp

# )
# # target_include_directories(${PROJECT_NAME}_test PRIVATE
# #   test/include
# # )
# target_link_libraries(${PROJECT_NAME}_test
#   gtest_main
#   ${PROJECT_NAME}

# )
# target_compile_options(${PROJECT_NAME}_test PRIVATE ${FLAGS})
