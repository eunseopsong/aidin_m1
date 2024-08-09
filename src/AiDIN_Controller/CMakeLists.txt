include_directories("./")
include_directories("./Controllers")
# include_directories("../../robot/include")
# include_directories(SYSTEM "../../third-party/qpOASES/include")

include_directories("/usr/local/include/eigen3")   # lcm includes
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

file(GLOB_RECURSE sources
"./*.cpp"
"FSM_States/*.cpp"
"Controllers/convexMPC/*.cpp")

# add_subdirectory(Controllers/WBC)

add_executable(aidin_ctrl ${sources} main.cpp)

# target_link_libraries(mit_ctrl robot biomimeti/cs)
# target_link_libraries(mit_ctrl WBC_Ctrl)
# target_link_libraries(mit_ctrl VisionMPC)
