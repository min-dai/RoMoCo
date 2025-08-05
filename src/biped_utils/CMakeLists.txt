cmake_minimum_required(VERSION 3.5.0)
project(biped_utils)

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
find_package(yaml_cpp_vendor REQUIRED)
find_package(Eigen3 3.4 REQUIRED)

set(dependencies
  rclcpp
  yaml_cpp_vendor
  Eigen3
)

# Library target
add_library(${PROJECT_NAME} SHARED
    src/bezier_tools.cpp
    src/PhaseVariable.cpp
    src/geometry.cpp
    src/hyperbolic.cpp
    src/algebraic_riccati.cpp
    src/Pinverse.cpp
    src/yaml_parser.cpp
    src/angular_momentum_kf.cpp)
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

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test 
    test/test_algebraic_riccati.cpp
    test/test_bezier_tools.cpp
    test/test_yaml_parser.cpp
    test/test_kinematics.cpp
  )
  target_include_directories(${PROJECT_NAME}_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
endif()


ament_package()
