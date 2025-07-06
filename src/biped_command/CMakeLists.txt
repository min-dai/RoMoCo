cmake_minimum_required(VERSION 3.5.0)
project(biped_command)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(CATKIN_DEPS
  roscpp
  std_msgs
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPS}
)

find_package(Eigen3 3.4 REQUIRED)
find_package(Qt5Widgets REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_DEPS}
  DEPENDS Qt5Widgets Eigen3
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
)

qt5_wrap_cpp(SLIDER_MOC_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/include/biped_command/radio_slider_gui.hpp)

add_library(${PROJECT_NAME}
    src/radio_slider_gui.cpp
  ${SLIDER_MOC_SRCS})


target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  Eigen3::Eigen
  Qt5::Widgets
  Qt5::Core
)

## Ros Node
add_executable(fake_radio_node src/radio_main_node.cpp)
target_link_libraries(fake_radio_node ${PROJECT_NAME})

## Test
catkin_add_gtest(${PROJECT_NAME}_test
  test/test_slider_gui.cpp
)
# target_include_directories(${PROJECT_NAME}_test PRIVATE
#   test/include
# )
target_link_libraries(${PROJECT_NAME}_test
  gtest_main
  ${PROJECT_NAME}
)
target_compile_options(${PROJECT_NAME}_test PRIVATE ${FLAGS})


