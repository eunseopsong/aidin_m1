include_directories("./")

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_BINARY_DIR}
  ${GAZEBO_INCLUDE_DIRS}

)




file(GLOB_RECURSE sources
"./*.cpp"
)




add_executable(m1_plugin ${sources} aidin_m1_plugin.cpp)

