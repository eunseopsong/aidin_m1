include_directories("./")

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_BINARY_DIR}
  ${GAZEBO_INCLUDE_DIRS}

)

file(GLOB_RECURSE sources
"./*.cpp"
)

add_executable(m1_plugin ${sources})

#### 밑에 것들 없어도 되는 건가..? 왜 되지 ####
# build the plugin library
# add_library(${PROJECT_NAME}_plugin SHARED
#   src/gazebo_plugin/aidin_m1_plugin.cpp
# )

# ament_target_dependencies(${PROJECT_NAME}_plugin
#   "rclcpp"
#   "pluginlib"
#   "gazebo"
# )

# target_link_libraries(${PROJECT_NAME}_plugin
#   ${GAZEBO_LIBRARIES}
#   ${SDFORMAT_LIBRARIES}
#   ${qpOASES_LIBRARIES}
# )
