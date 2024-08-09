include_directories("./")

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_BINARY_DIR}
  ${GAZEBO_INCLUDE_DIRS}

)




file(GLOB_RECURSE sources
"./*.cpp"
)

# add_subdirectory(Controllers/WBC)

add_executable(m1_plugin ${sources} aidin_m1_plugin.cpp)
# target_link_libraries(mit_ctrl robot biomimeti/cs)
# target_link_libraries(mit_ctrl WBC_Ctrl)
# target_link_libraries(mit_ctrl VisionMPC)
