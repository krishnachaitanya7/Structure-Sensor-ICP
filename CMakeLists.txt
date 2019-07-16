cmake_minimum_required(VERSION 3.14)
project(Structure_Sensor_ICP)

set(CMAKE_CXX_STANDARD 14)
#add_executable(Structure_Sensor_ICP
#    depth_to_pcl/depth_to_pcl.cpp
#    get_depth_frames/get_depth_frames/get_depth_frames.cpp
#    get_depth_frames/get_depth_frames.cpp
#    ICP_test.cpp
#    main.cpp
#    show_ply.cpp)
# Below is the 3-4 lines you have to comment out when you want to have run other damn executables
# Fuck Me Please CMake
find_package(PCL REQUIRED)
add_executable(GetDepthFrames get_depth_frames/get_depth_frames.cpp)
#target_link_libraries(GetDepthFrames Structure)
target_link_libraries(GetDepthFrames ${PCL_LIBRARIES} Structure)
#ICP Test
#add_executable(icp_test ICP_test.cpp)
#target_link_libraries(icp_test ${PCL_LIBRARIES})
