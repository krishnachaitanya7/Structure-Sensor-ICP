cmake_minimum_required(VERSION 3.14)
project(Structure_Sensor_ICP)
set(CMAKE_CXX_STANDARD 14)
##New Feature Test
include(new_feature_test/CMakeLists.txt)
## Depth Frame Test
include(get_depth_frames/CMakeLists.txt)
#ICP Test
add_executable(icp_test ICP_test.cpp)
target_link_libraries(icp_test ${PCL_LIBRARIES})
