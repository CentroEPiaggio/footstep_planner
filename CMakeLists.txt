project(plane_segmentation)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED 
	roscpp 
	message_generation 
	geometry_msgs 
	std_msgs 
	shape_msgs
	sensor_msgs
	visualization_msgs
	tf
	)

find_package(PCL 1.7 REQUIRED)

add_message_files(
	FILES
	Plane.msg
	PlaneDetectionResult.msg
	)

add_service_files(
	FILES
	PlaneDetection.srv
	)

generate_messages(
	DEPENDENCIES
	geometry_msgs
	std_msgs
	shape_msgs
	sensor_msgs
	visualization_msgs
	)

catkin_package(
	)

include_directories(include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	)

link_directories(
	${PCL_LIBRARY_DIRS}
	)

add_definitions(
	${PCL_DEFINITIONS}
	)


add_executable(random_sample_consensus 
	src/random_sample_consensus.cpp
	)

target_link_libraries(random_sample_consensus 
	${catkin_LIBRARIES} 
	${PCL_LIBRARIES}
	)



add_library(PlaneMarker
	src/MarkerGenerator.cpp
	)

add_executable(plane_segmentation_node 
	src/plane_segmentation_node.cpp
	)

add_dependencies(plane_segmentation_node 
	plane_segmentation_gencpp
	PlaneMarker
	)

target_link_libraries(plane_segmentation_node 
	PlaneMarker
	${catkin_LIBRARIES} 
	${PCL_LIBRARIES}
	)


add_executable(curvature_filter_node 
	src/curvature_filter_node.cpp
	)

target_link_libraries(curvature_filter_node
	${catkin_LIBRARIES} 
	${PCL_LIBRARIES}
	)