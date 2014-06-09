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
#include <drc_shared/idynutils.h>
// class object for the ROS node
namespace planner {

class rosServer
{
public:
    rosServer(ros::NodeHandle nh);
  private:
    //! The node handle
    ros::NodeHandle nh_;

    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    //! Publishers
    ros::Publisher pub_cluster_cloud_;
    ros::Publisher pub_border_poly_marker;
    ros::Publisher pub_footstep;
    
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
    ros::Publisher pub_ik_joints;
    std::string camera_link_name;
    
  public:
    //------------------ Callbacks -------------------
    bool filterByCurvature(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool extractBorders(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool planFootsteps(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    void run();
    
    std::vector< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr > clusters;
    std::vector<  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >> polygons;
    std::vector <geometry_msgs::Pose> footsteps;
    
};




} // namespace plane_segmentation