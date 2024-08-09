include_directories("./")
include_directories("./Controllers")

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFORMAT_INCLUDE_DIRS}
  ${qpOASES_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_BINARY_DIR}
)

file(GLOB_RECURSE sources
"./*.cpp"
"FSM_States/*.cpp"
"Controllers/convexMPC/*.cpp")

add_executable(main ${sources})

# Link the necessary libraries to the executable
target_link_libraries(main
  ${GAZEBO_LIBRARIES}
  ${SDFORMAT_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${qpOASES_LIBRARIES}
)

# Optionally install the executable
install(TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)
