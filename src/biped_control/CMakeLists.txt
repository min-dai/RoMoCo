# cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
cmake_minimum_required(VERSION 3.0)
project(biped_control)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CATKIN_DEPS
  roscpp
  biped_utils
  biped_core
  biped_command
  biped_planner
)


find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPS}
)



###########
## Build ##
###########


# Specify custom includes and libs
include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS ${CATKIN_DEPS}
   DEPENDS 
)


# Create controller libraries
add_library(${PROJECT_NAME}
    src/inair_output.cpp
    src/standing_output.cpp
    src/walking_output_base.cpp
    src/walking_output_fp.cpp
    # src/walking_output_multidomain_fp.cpp
)  
        

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})




# catkin_add_gtest(${PROJECT_NAME}_test
#   test/test_basic.cpp
# )
# target_include_directories(${PROJECT_NAME}_test PRIVATE
#   test/include
# )
# target_link_libraries(${PROJECT_NAME}_test
#   gtest_main
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )
# target_compile_options(${PROJECT_NAME}_test PRIVATE ${FLAGS})
