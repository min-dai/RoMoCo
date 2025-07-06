cmake_minimum_required(VERSION 3.5.0)
project(biped_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp  
)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.4 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp  
  DEPENDS yaml-cpp Eigen3
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
    src/bezier_tools.cpp
    src/PhaseVariable.cpp
    src/geometry.cpp
    src/hyperbolic.cpp
    src/algebraic_riccati.cpp
    src/Pinverse.cpp
    src/yaml_parser.cpp
    src/angular_momentum_kf.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)


## Test
catkin_add_gtest(${PROJECT_NAME}_test
  test/test_algebraic_riccati.cpp
  test/test_bezier_tools.cpp
)
target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}
  gtest_main
)
target_compile_options(${PROJECT_NAME}_test PRIVATE ${FLAGS})