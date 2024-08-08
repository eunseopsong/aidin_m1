include_directories("./")
# include_directories("../../robot/include")
# include_directories(SYSTEM "../../third-party/qpOASES/include")

include_directories("/usr/local/include/eigen3")   # lcm includes
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

file(GLOB_RECURSE sources
"./*.cpp"
)

# add_subdirectory(Controllers/WBC)

add_executable(m1_plugin ${sources} aidin_m1_plugin.cpp)
# target_link_libraries(mit_ctrl robot biomimeti/cs)
# target_link_libraries(mit_ctrl WBC_Ctrl)
# target_link_libraries(mit_ctrl VisionMPC)
