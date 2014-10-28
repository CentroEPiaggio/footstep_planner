/* Copyright [2014] [Mirko Ferrati, Alessandro Settimi, Corrado Pavan, Carlos J Rosales]
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/


// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <ros/publisher.h>
#include <tf/transform_listener.h>

// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/boundary.h>
#include <pcl/io/io.h>

#include "footstep_planner.h"
#include "curvaturefilter.h"
#include "borderextraction.h"
#include "ros_publisher.h"
#include <drc_shared/yarp_msgs/fs_walking_msg.h>
#include <drc_shared/idynutils.h>

#include <yarp/os/all.h>
#include <drc_shared/yarp_status_interface.h>
#include <drc_shared/yarp_command_interface.hpp>
#include <drc_shared/yarp_msgs/fs_planner_msg.h>
#include <drc_shared/yarp_single_chain_interface.h>

// class object for the ROS node
namespace planner {

class rosServer: public yarp::os::RateThread
{
public:
    rosServer(ros::NodeHandle* nh_,yarp::os::Network* yarp_, double period,std::string robot_name_);
  private:
    //! The node handle
    ros::NodeHandle* nh;
    yarp::os::Network* yarp;

    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    //! Publishers
    ros_publisher publisher;
    
    //! Subscribers
    ros::Subscriber sub_input_cloud_;
    tf::TransformListener listener;
    
    //! Services
    ros::ServiceServer srv_filter_cloud_;
    ros::ServiceServer srv_border_extraction;
    ros::ServiceServer srv_footstep_placer;
    
    footstepPlanner footstep_planner;
    curvatureFilter curvature_filter;
    borderExtraction border_extraction;
    tf::Transform current_robot_transform;

    
    walkman::drc::yarp_single_chain_interface left_leg,right_leg,left_arm,right_arm,torso;
    
    //Camera link frame
    KDL::Vector current_direction;
    std::string camera_link_name;
    std::string filename;
    //WORLD Reference Frame
    std::vector<std::pair<foot_with_joints,std::vector<std::string>>> path;
    
    bool singleFoot(bool left);
    
    walkman::drc::yarp_custom_command_interface<fs_planner_msg> command_interface;
    walkman::drc::yarp_custom_command_sender_interface<fs_walking_msg> walking_command_interface;
    int seq_num_out=0;
    fs_planner_msg msg;
    walkman::drc::yarp_status_interface status_interface;
    bool left;
    bool save_to_file;
    void setInitialPosition();
    std::map<std::string,double> initial_joints_value;
    bool single_check(bool ik_only, bool move);
    
  public:
    //------------------ Callbacks -------------------
    bool filterByCurvature(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool extractBorders(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool planFootsteps(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool sendPathToRviz();
    bool create_steps_vector(fs_walking_msg& temp);
    void init();
    
    virtual bool threadInit();
    virtual void run();
    
    //Camera_link reference Frame
    std::vector< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > clusters;
    std::list< polygon_with_normals > polygons;
    
};




} // namespace plane_segmentation
