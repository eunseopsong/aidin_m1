cmake_minimum_required(VERSION 3.8)
project(aidin_m1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_path(qpOASES_INCLUDE_DIR qpOASES.hpp)
find_library(qpOASES_LIBRARIES qpOASES)

if(NOT qpOASES_INCLUDE_DIR OR NOT qpOASES_LIBRARIES)
  message(FATAL_ERROR "Could not find qpOASES")
endif()

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${qpOASES_INCLUDE_DIR}
)

# the below 4 sentences are neccessary??
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install worlds directory
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# include the sub directory
add_subdirectory(src)
add_subdirectory(common)
add_subdirectory(third-party)

ament_package()
