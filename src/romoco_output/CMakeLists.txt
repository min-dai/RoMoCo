cmake_minimum_required(VERSION 3.5.0)
project(romoco_output)

# Humble Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(romoco_utils REQUIRED)
find_package(romoco_core REQUIRED)
find_package(romoco_planner REQUIRED)

set(dependencies
  Eigen3
  romoco_utils
  romoco_core
  romoco_planner
)

# Library target
add_library(${PROJECT_NAME} SHARED
    src/inair_output.cpp
    src/standing_output.cpp
    src/walking_output_base.cpp
    src/walking_output_fp.cpp
    src/com_states_rel_to_stance.cpp
)  
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
    test/test_basic.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test 
    ${PROJECT_NAME}
  )

  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
endif()


ament_package()
