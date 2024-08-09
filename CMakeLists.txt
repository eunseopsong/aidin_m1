cmake_minimum_required(VERSION 3.8)
project(aidin_m1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(sdformat9 REQUIRED)
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
  ${SDFORMAT_INCLUDE_DIRS}
  ${qpOASES_INCLUDE_DIR}
)

# the below 4 sentences are neccessary??
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


# build the plugin library
add_library(${PROJECT_NAME}_plugin SHARED
  src/gazebo_plugin/aidin_m1_plugin.cpp
)

ament_target_dependencies(${PROJECT_NAME}_plugin
  "rclcpp"
  "pluginlib"
  "gazebo"
)

target_link_libraries(${PROJECT_NAME}_plugin
  ${GAZEBO_LIBRARIES}
  ${SDFORMAT_LIBRARIES}
  ${qpOASES_LIBRARIES}
)

ament_package()
