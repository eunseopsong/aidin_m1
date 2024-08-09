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



add_executable(aidin_ctrl ${sources} main.cpp)
