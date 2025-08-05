cmake_minimum_required(VERSION 3.5.0)
project(biped_command)

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
find_package(std_msgs REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(dependencies 
  rclcpp
  std_msgs
  Eigen3
  Qt5Widgets
)

# Library target
qt5_wrap_cpp(SLIDER_MOC_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/include/biped_command/radio_slider_gui.hpp)

add_library(${PROJECT_NAME} SHARED
    src/radio_slider_gui.cpp
  ${SLIDER_MOC_SRCS})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
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
add_executable(fake_radio_node src/radio_main_node.cpp)
target_link_libraries(fake_radio_node ${PROJECT_NAME})
install(
  TARGETS fake_radio_node
  DESTINATION lib/${PROJECT_NAME}
)




# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test 
    test/test_slider_gui.cpp
  )
  target_include_directories(${PROJECT_NAME}_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
endif()


ament_package()



