# Include directories
include_directories("./")

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${qpOASES_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_BINARY_DIR}
)

# Gather all source files recursively
file(GLOB_RECURSE sources
  "./*.cpp"
)

# Define the plugin library target
add_library(aidin_m1_plugin SHARED ${sources})

# Link the necessary libraries to the plugin
target_link_libraries(aidin_m1_plugin
  ${GAZEBO_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${SDFORMAT_LIBRARIES}
  ${qpOASES_LIBRARIES}
)

# Specify ament dependencies
ament_target_dependencies(aidin_m1_plugin
  "rclcpp"
  "pluginlib"
  "gazebo"
)

# Install the Gazebo plugin library
install(TARGETS aidin_m1_plugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)
