// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>


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

#include "psimpl.h"
#include <eigen3/Eigen/Eigen>

// class object for the ROS node
namespace plane_segmentation {

class CurvatureFilter
{
  private:
    //! The node handle
    ros::NodeHandle nh_;

    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    //! Publishers
    ros::Publisher pub_colored_cloud_;
    ros::Publisher pub_filtered_cloud_;
    ros::Publisher pub_cluster_cloud_;
    ros::Publisher pub_polygons_marker;
    ros::Publisher pub_border_marker;
    ros::Publisher pub_border_poly_marker;
    ros::Publisher pub_footstep;
    
    //! Subscribers
    ros::Subscriber sub_input_cloud_;

    //! Services
    ros::ServiceServer srv_filter_cloud_;
    
    ros::ServiceServer srv_convex_hull;

    ros::ServiceServer srv_border_extraction;
    
    ros::ServiceServer srv_footstep_placer;
    
    // downsample
    double voxel_size_;

    // normal estimator radius
    double normal_radius_;

    // normal estimator radius
    double curvature_threshold_;

    // euclidean cluster min size (related to the area, since we are filtering only planar regions on a grid)
    int min_cluster_size_;
    
    // euclidean cluster tolerance
    double cluster_tolerance_;

    std::vector< pcl::PointCloud<pcl::PointXYZRGBNormal> > clusters;
    
    std::vector< pcl::PointCloud<pcl::PointXYZ> > polygons;
    
  public:
    //------------------ Callbacks -------------------
    // Callback for filtering the cloud
    // void filterByCurvature(const sensor_msgs::PointCloud2 & input);

    bool filterByCurvature(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    
    bool convex_hull(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    bool border_extraction(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    
    bool douglas_peucker_3d(pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& output, double tolerance=10);
    
    bool footstep_placer(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    
    bool centroid_is_reachable(Eigen::Matrix<double,4,1> centroid);
    
    bool step_is_stable(Eigen::Matrix< double, 4, 1 > centroid);
    
    bool plane_is_compatible(Eigen::Matrix< double, 4, 1 > centroid);
    
    std::vector< pcl::PointCloud<pcl::PointXYZ> > compute_polygon_grid(pcl::PointCloud<pcl::PointXYZ> polygon);
    
    std::vector <geometry_msgs::Pose> footsteps;
    
    //! Subscribes to and advertises topics
    CurvatureFilter(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
      // init publishers and subscribers
      pub_colored_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(nh_.resolveName("colored_cloud"), 100);
      pub_filtered_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(nh_.resolveName("filtered_cloud"), 100);
      pub_cluster_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(nh_.resolveName("cluster_cloud"), 3000);
      pub_polygons_marker = nh_.advertise<visualization_msgs::Marker>("/polygons_marker",1,this);
      pub_border_marker = nh_.advertise<visualization_msgs::Marker>("/border_marker",1,this);
      pub_border_poly_marker = nh_.advertise<visualization_msgs::Marker>("/border_poly_marker",1,this);
      pub_footstep = nh_.advertise<visualization_msgs::Marker>("/footstep_marker",100,this);

      //sub_input_cloud_ = nh_.subscribe(nh_.resolveName("input_cloud"), 100, &CurvatureFilter::filterByCurvature, this);

      srv_filter_cloud_ = nh_.advertiseService(nh_.resolveName("filter_by_curvature"), &CurvatureFilter::filterByCurvature, this);
      srv_convex_hull = nh_.advertiseService(nh_.resolveName("convex_hull"), &CurvatureFilter::convex_hull, this);
      srv_border_extraction = nh_.advertiseService(nh_.resolveName("border_extraction"), &CurvatureFilter::border_extraction, this);
      srv_footstep_placer = nh_.advertiseService(nh_.resolveName("footstep_placer"),&CurvatureFilter::footstep_placer, this);
      
      priv_nh_.param<double>("voxel_size", voxel_size_, 0.02);
      priv_nh_.param<double>("normal_radius", normal_radius_, 0.05);
      priv_nh_.param<double>("curvature_threshold", curvature_threshold_, 0.01);
      priv_nh_.param<int>("min_cluster_size", min_cluster_size_, 100);
      priv_nh_.param<double>("cluster_tolerance", cluster_tolerance_, 100);
      

    }

    //! Empty stub
    ~CurvatureFilter() {}

    std::map< int , std::vector<uint8_t> > color_map;
    
};

} // namespace plane_segmentation