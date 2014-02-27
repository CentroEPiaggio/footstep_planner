cmake_minimum_required(VERSION 2.8.3)
project(plane_segmentation)

find_package(catkin REQUIRED)

catkin_package()

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(random_sample_consensus random_sample_consensus.cpp)

target_link_libraries(random_sample_consensus ${catkin_LIBRARIES} ${PCL_LIBRARIES})