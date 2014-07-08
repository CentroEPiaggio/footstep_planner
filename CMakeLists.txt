cmake_minimum_required(VERSION 2.8.3)
project(footstep_planner)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 ")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
 roscpp 
        message_generation 
        geometry_msgs 
        std_msgs 
        shape_msgs
        sensor_msgs
        visualization_msgs
        tf
	tf_conversions
	urdf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(iDynTree REQUIRED)
find_package(drc_shared REQUIRED)
find_package(kdl_codyco REQUIRED)
find_package(TinyXML REQUIRED)

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
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy roslib urdf srdfdom
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${YARP_INCLUDE_DIRS} ${IDYN3_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${drc_shared_INCLUDE_DIRS})
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

FILE(GLOB HEADER_FILES include/*.h)

add_executable(foot_planner
       src/gram_schmidt.cpp
       src/ros_publisher.cpp
       src/ros_server.cpp
       src/footstep_planner.cpp
       src/kinematics_utilities.cpp
       src/curvaturefilter.cpp
       src/borderextraction.cpp
       src/kinematic_filter.cpp
       src/com_filter.cpp
       src/step_quality_evaluator.cpp
       src/coordinate_filter.cpp
       src/main.cpp
        ${HEADER_FILES}
       )

add_executable(affordances
        src/affordances_main.cpp
        src/gram_schmidt.cpp
        src/ros_publisher.cpp
        src/ros_server.cpp
        src/footstep_planner.cpp
        src/kinematics_utilities.cpp
        src/curvaturefilter.cpp
        src/borderextraction.cpp
        src/kinematic_filter.cpp
        src/com_filter.cpp
        src/step_quality_evaluator.cpp
        ${HEADER_FILES}
)

target_link_libraries(foot_planner
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${drc_shared_LIBRARIES}
         ${iDynTree_LIBRARIES}
        )

target_link_libraries(affordances
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${drc_shared_LIBRARIES}
        ${iDynTree_LIBRARIES}
        ${TinyXML_LIBRARIES}
        )

## Specify libraries to link a library or executable target against
# target_link_libraries(yarp_ros_interface ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES} ${catkin_LIBRARIES})
# target_link_libraries(yarp_ros_joint_state_publisher_node yarp_ros_interface ${YARP_LIBRARIES} icubmod                        ${ICUB_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############


## Mark executables and/or libraries for installation
install(TARGETS foot_planner
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS affordances
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  
install(DIRECTORY resources/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources)

