cmake_minimum_required(VERSION 3.5.0)
project(mujoco_interface)

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


set(MUJOCO_DIR $ENV{HOME}/repos/mujoco-3.2.6)
set(MUJOCO_LIBRARY ${MUJOCO_DIR}/lib/libmujoco.so)
set(GLFW_INCLUDE_DIR /usr/include)
set(GLFW_LIBRARY /usr/lib/x86_64-linux-gnu/libglfw.so)

set(dependencies 
  rclcpp
  Eigen3
)

set(include_dirs 
  ${MUJOCO_DIR}/include
  ${GLFW_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

# Library target
add_library(${PROJECT_NAME}
   src/mujoco_interface.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${include_dirs}
)
target_link_libraries(${PROJECT_NAME} 
  ${MUJOCO_LIBRARY}
  ${GLFW_LIBRARY}
)
ament_target_dependencies(${PROJECT_NAME} 
  ${dependencies}
)

#todo
set_target_properties(${PROJECT_NAME} PROPERTIES
  INSTALL_RPATH "$ENV{HOME}/repos/mujoco-3.2.6/lib"
)


# Install headers and library
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_export_include_directories(${include_dirs})
ament_export_libraries(${PROJECT_NAME} ${MUJOCO_LIBRARY} ${GLFW_LIBRARY})
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
    test/test_mujoco_interface.cpp
  )
  target_include_directories(${PROJECT_NAME}_test PRIVATE
    ${include_dirs}
  )
  target_link_libraries(${PROJECT_NAME}_test 
    ${PROJECT_NAME}
    ${MUJOCO_LIBRARY}
    ${GLFW_LIBRARY}
  )

  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
endif()


ament_package()

