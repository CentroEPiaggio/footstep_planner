# Copyright: (C) Mirko Ferrati, Alessandro Settimi, Corrado Pavan, Carlos Rosales
# Authors: Mirko Ferrati, Alessandro Settimi, Corrado Pavan, Carlos Rosales
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 2.8.3)
project(footstep_planner)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
endif(COMMAND cmake_policy)
include(ExternalProject)
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
# find_package(YARP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
# find_package(iDynTree REQUIRED)
# find_package(TinyXML REQUIRED)
# find_package(drc_shared REQUIRED)
# find_package(idynutils REQUIRED)
# find_package(GYM REQUIRED)
# find_package(paramHelp REQUIRED)
# find_package(kdl_codyco REQUIRED)
# FIND_PACKAGE(tf REQUIRED)
# FIND_PACKAGE(tf_conversions REQUIRED)
# FIND_PACKAGE(urdf REQUIRED)
# FIND_PACKAGE(gencpp REQUIRED)

# FIND_PACKAGE(roscpp REQUIRED)
# FIND_PACKAGE(message_generation REQUIRED)

FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp rospy roslib urdf srdfdom message_runtime message_generation std_msgs sensor_msgs kdl_parser tf_conversions urdf gencpp 
)
FIND_PACKAGE(orocos_kdl REQUIRED)

#set(ROS_LIB_FOLDER $ENV{ROS_LIB_FOLDER} CACHE STRING "ros lib folder")

#LINK_DIRECTORIES(${ROS_LIB_FOLDER} )
# add_message_files(
# FILES
# fs_command.msg
# )
# 
add_service_files(
FILES
fs.srv
)

generate_messages(
DEPENDENCIES std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy roslib urdf srdfdom message_runtime message_generation std_msgs sensor_msgs kdl_parser tf_conversions urdf gencpp
  DEPENDS orocos_kdl
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include #${YARP_INCLUDE_DIRS} ${IDYN3_INCLUDE_DIRS} 
                            #${ICUB_INCLUDE_DIRS} 
                            ${Eigen_INCLUDE_DIRS} 
			    ${orocos_kld_INCLUDE_DIRS}
#                             ${TinyXML_INCLUDE_DIRS}
#                            ${drc_shared_INCLUDE_DIRS}
#                             ${GYM_INCLUDE_DIRS} ${idynutils_INCLUDE_DIRS}
                            )
include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        )

link_directories(
        ${PCL_LIBRARY_DIRS}
        )

add_definitions(
        ${PCL_DEFINITIONS} -DBOOST_NO_CXX11_EXPLICIT_CONVERSION_OPERATORS
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
       src/lipm_filter.cpp
       src/step_quality_evaluator.cpp
       src/coordinate_filter.cpp
       src/foot_collision_filter.cpp
       src/tilt_filter.cpp
       src/main.cpp
       src/xml_pcl_io.cpp
       src/pcd_io.cpp
       src/joints_ordering.cpp
       src/sampling_surface.hpp
       src/param_manager.cpp
       ${HEADER_FILES}
       )

target_link_libraries(foot_planner
       ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
	${orocos_kdl_LIBRARIES}
#         ${drc_shared_LIBRARIES}
#         ${idynutils_LIBRARIES}
#         ${GYM_LIBRARIES}
#         ${iDynTree_LIBRARIES}
#         ${urdf_LIBRARIES}
#         ${tf_LIBRARIES}
#         ${tf_conversions_LIBRARIES}
#         ${TinyXML_LIBRARIES}
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



install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  
install(DIRECTORY resources/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources)


OPTION(COMPILE_XML_IO_TEST "Build a test executables with xml input/output" OFF)
if(COMPILE_XML_IO_TEST)
add_executable(affordances
        src/pcd_io.cpp
        src/affordances_main.cpp
        src/gram_schmidt.cpp
        src/ros_publisher.cpp
        src/ros_server.cpp
        src/footstep_planner.cpp
        src/kinematics_utilities.cpp
        src/curvaturefilter.cpp
        src/foot_collision_filter.cpp
        src/borderextraction.cpp
        src/kinematic_filter.cpp
        src/com_filter.cpp
        src/step_quality_evaluator.cpp
        src/coordinate_filter.cpp
        src/tilt_filter.cpp
        src/xml_pcl_io.cpp
        ${HEADER_FILES}
)

target_link_libraries(affordances
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${drc_shared_LIBRARIES}
        ${iDynTree_LIBRARIES}
        ${TinyXML_LIBRARIES}
        ${urdf_LIBRARIES}
        ${tf_LIBRARIES}
        ${tf_conversions_LIBRARIES}
        )


install(TARGETS affordances
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endif()
