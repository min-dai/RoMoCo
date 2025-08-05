cmake_minimum_required(VERSION 3.5.0)
project(torque_control)



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
find_package(biped_core REQUIRED)


set(clarabel_LIB $ENV{HOME}/repos/Clarabel.cpp/rust_wrapper/target/release/libclarabel_c.so)
set(clarabel_INCLUDE_DIR $ENV{HOME}/repos/Clarabel.cpp/include/) 

set(dependencies 
  rclcpp
  Eigen3
  biped_core
)
set(include_dirs
  ${EIGEN3_INCLUDE_DIR}
  ${clarabel_INCLUDE_DIR}
)

# Library target
add_library(${PROJECT_NAME}
  src/torque_solver_base.cpp
  src/torque_solver_tscqp.cpp
  src/pd_controller.cpp
  src/torque_solver_inv_dyn.cpp
  src/torque_solver_vel_ik.cpp
  src/torque_solver_pos_ik.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${include_dirs}
)
target_link_libraries(${PROJECT_NAME} 
  ${clarabel_LIB}
)
ament_target_dependencies(${PROJECT_NAME} 
  ${dependencies}
)

# Install headers and library
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_export_include_directories(${include_dirs})
ament_export_libraries(${PROJECT_NAME} ${clarabel_LIB})
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
    test/test_clarabel.cpp
  )
  target_include_directories(${PROJECT_NAME}_test PRIVATE
    ${include_dirs}
  )
  target_link_libraries(${PROJECT_NAME}_test 
    ${PROJECT_NAME}
    ${clarabel_LIB}
  )

  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
endif()


ament_package()

