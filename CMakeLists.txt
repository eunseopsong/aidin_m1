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

# 추가: qpOASES 라이브러리를 찾음
find_path(qpOASES_INCLUDE_DIR qpOASES.hpp)
find_library(qpOASES_LIBRARIES qpOASES)

if(NOT qpOASES_INCLUDE_DIR OR NOT qpOASES_LIBRARIES)
  message(FATAL_ERROR "Could not find qpOASES")
endif()

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFORMAT_INCLUDE_DIRS}
  ${qpOASES_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/gazebo_plugin
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

list(APPEND CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

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

# Build the main executable
# Add other source files containing definitions for the functions and variables used in main.cpp
add_executable(main src/AiDIN_Controller/main.cpp src/AiDIN_Controller/JointControl.cpp src/AiDIN_Controller/func.cpp src/AiDIN_Controller/Controllers/convexMPC/convexMPC.cpp)

target_link_libraries(main
  ${GAZEBO_LIBRARIES}
  ${SDFORMAT_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${qpOASES_LIBRARIES}
)

# Install
install(DIRECTORY launch worlds urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install the plugin library
install(TARGETS ${PROJECT_NAME}_plugin
  DESTINATION lib
)

# Install the main executable
install(TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
