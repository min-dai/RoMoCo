cmake_minimum_required(VERSION 3.5.0)
project(biped_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp  
  biped_utils
)

find_package(Eigen3 3.4 REQUIRED)

find_package(pinocchio REQUIRED)
message("Pinocchio include dirs: ${PINOCCHIO_INCLUDE_DIRS}")

# Specify custom includes and libs
include_directories(include 
                    ${catkin_INCLUDE_DIRS}
                    ${PINOCCHIO_INCLUDE_DIRS} 
                    )

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp biped_utils
   DEPENDS  pinocchio
   CFG_EXTRAS 
)


add_library(${PROJECT_NAME} SHARED 
            src/output_base.cpp
            src/robot_base_pinocchio.cpp
            src/outputs.cpp
            src/planefoot_robot_base_pinocchio.cpp
            src/linefoot_robot_base_pinocchio.cpp
            src/com_states_rel_to_stance.cpp
            )

target_link_libraries(${PROJECT_NAME} 
                        ${catkin_LIBRARIES} 
                        pinocchio::pinocchio
                        Eigen3::Eigen
                        )

