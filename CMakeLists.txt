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
  ${qpOASES_INCLUDE_DIR} # 추가: qpOASES include 디렉토리
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

list(APPEND CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install the launch directory
install(DIRECTORY
  launch
  meshes
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Build
add_executable(aidin_m1_main src/aidin_m1_main.cpp)
ament_target_dependencies(aidin_m1_main rclcpp std_msgs)

add_library(aidin_m1_plugin SHARED src/aidin_m1_plugin.cpp)
target_link_libraries(aidin_m1_plugin
  ${GAZEBO_LIBRARIES}
  ${SDFORMAT_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${qpOASES_LIBRARIES} # 추가: qpOASES 라이브러리
)
ament_target_dependencies(aidin_m1_plugin rclcpp std_msgs)

# Install
install(TARGETS
  aidin_m1_main
  aidin_m1_plugin
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
# push test