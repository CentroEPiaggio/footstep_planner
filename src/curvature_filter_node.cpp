// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Empty.h>

// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <visualization_msgs/Marker.h>

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

    //! Subscribers
    ros::Subscriber sub_input_cloud_;

    //! Services
    ros::ServiceServer srv_filter_cloud_;
    
    ros::ServiceServer srv_footstep_place;

    // downsample
    double voxel_size_;

    // normal estimator radius
    double normal_radius_;

    // normal estimator radius
    double curvature_threshold_;

    // euclidean cluster min size (related to the area, since we are filtering only planar regions on a grid)
    int min_cluster_size_;
    
    std::vector< pcl::PointCloud<pcl::PointXYZRGBNormal> > clusters;
    
  public:
    //------------------ Callbacks -------------------
    // Callback for filtering the cloud
    // void filterByCurvature(const sensor_msgs::PointCloud2 & input);
    bool filterByCurvature(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    
    bool footstep_place(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    //! Subscribes to and advertises topics
    CurvatureFilter(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
   		// init publishers and subscribers
		  pub_colored_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(nh_.resolveName("colored_cloud"), 100);
      pub_filtered_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(nh_.resolveName("filtered_cloud"), 100);
      pub_cluster_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(nh_.resolveName("cluster_cloud"), 3000);
      pub_polygons_marker = nh_.advertise<visualization_msgs::Marker>("/polygons_marker",1,this);
      
     	//sub_input_cloud_ = nh_.subscribe(nh_.resolveName("input_cloud"), 100, &CurvatureFilter::filterByCurvature, this);

      srv_filter_cloud_ = nh_.advertiseService(nh_.resolveName("filter_by_curvature"), &CurvatureFilter::filterByCurvature, this);
      srv_footstep_place = nh_.advertiseService(nh_.resolveName("footstep_place"), &CurvatureFilter::footstep_place, this);

      priv_nh_.param<double>("voxel_size", voxel_size_, 0.02);
      priv_nh_.param<double>("normal_radius", normal_radius_, 0.05);
      priv_nh_.param<double>("curvature_threshold", curvature_threshold_, 0.01);
      priv_nh_.param<int>("min_cluster_size", min_cluster_size_, 100);
      

    }

    //! Empty stub
    ~CurvatureFilter() {}

    std::map< int , std::vector<uint8_t> > color_map;
    
};

//void CurvatureFilter::filterByCurvature(const sensor_msgs::PointCloud2 & input)
bool CurvatureFilter::filterByCurvature(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{

  ros::Time start_time = ros::Time::now();

  // wait for a point cloud
  std::string topic = nh_.resolveName("/camera/depth_registered/points");
  ROS_INFO("waiting for a point_cloud2 on topic %s", topic.c_str());
  sensor_msgs::PointCloud2::ConstPtr input = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh_, ros::Duration(3.0));
  if (!input)
  {
    ROS_ERROR("no point_cloud2 has been received");
    return false;
  }

  // convert from sensor_msgs to a tractable PCL object
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*input, *input_cloud_ptr);
  //ROS_INFO("Cloud size is %li", input_cloud_ptr->points.size());

  //remove nans
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> nans;
  pcl::removeNaNFromPointCloud(*input_cloud_ptr, *cloud_ptr, nans);

	pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  grid.setLeafSize (voxel_size_, voxel_size_, voxel_size_);
	grid.setFilterFieldName ("z");
	grid.setDownsampleAllData (false);
	grid.setInputCloud (cloud_ptr);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	grid.filter (*cloud_downsampled_ptr);

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud = *cloud_downsampled_ptr;

  // compute normals and curvature
  //pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr (new pcl::PointCloud<pcl::Normal> ());

  ne.setInputCloud (cloud_downsampled_ptr);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (normal_radius_);
  ne.compute (*cloud_normals_ptr);
  
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
  pcl::concatenateFields( *cloud_downsampled_ptr, *cloud_normals_ptr, *cloud_with_normals_ptr );
  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_with_normals;
  cloud_with_normals = *cloud_with_normals_ptr;


  int N = (int) cloud_with_normals.points.size();
  ROS_INFO("Cloud size is %i", N);

  // find max curvature to scale the color
  float c, c_max = 0.0;
  // for (int i = 0; i < N; i++)
  // {
  //   c = cloud_with_normals.points[i].curvature;
  //   if (c > c_max)
  //   {
  //     c_max = c;
  //   }
  // }
  // ROS_INFO("Maximum curvature in cloud is %f", c_max);
  // c_max = 1/c_max;

  // for coloring the clouds
  uint8_t r, g, b;
  int32_t rgb; 

  std::vector<uint8_t> color_vec;
  color_vec.push_back(0);color_vec.push_back(0);color_vec.push_back(0);
  
  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_with_low_curvature;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_low_curvature_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
  // filter and colour the cloud according to the curvature
  for (int i = 0; i < N; i++)
  {
  	c = cloud_with_normals.points[i].curvature;
    //ROS_INFO("Curvature %f", c);
    if (c > curvature_threshold_)
    {
      r = 255;//*(c*c_max);
      g = 0;//255*(1 - c*c_max);
      b = 0;
      rgb = (r << 16) | (g << 8) | b; 
      cloud_with_normals.points[i].rgb = *(float *)(&rgb);
    }
    else
    {
      r = 0;//255*(c*c_max);
      g = 255;//*(1 - c*c_max);
      b = 0;
      rgb = (r << 16) | (g << 8) | b; 
      cloud_with_normals.points[i].rgb = *(float *)(&rgb);

      // keep this ones in a separate cloud
      cloud_with_low_curvature_ptr->points.push_back(cloud_with_normals.points[i]);
    }
  }

  // publish the colored cloud
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_with_normals, cloud_msg);
  pub_colored_cloud_.publish(cloud_msg);

  cloud_with_low_curvature = *cloud_with_low_curvature_ptr;
  // publish the cloud with planar regions only
  sensor_msgs::PointCloud2 another_cloud_msg;
  pcl::toROSMsg(cloud_with_low_curvature, another_cloud_msg);
  another_cloud_msg.header = cloud_msg.header;
  pub_filtered_cloud_.publish(another_cloud_msg);


  // perform euclidean clustering on the cloud with planar areas only
  // we need another tree because this cloud is different
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr another_tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormal> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (min_cluster_size_); // related to the area since we are in a planar grid
  ec.setMaxClusterSize (2500000);
  ec.setSearchMethod (another_tree);
  ec.setInputCloud (cloud_with_low_curvature_ptr);
  ec.extract (cluster_indices);

  ROS_INFO( "Found %lu clusters", cluster_indices.size() );

  int Ncluster = (int)cluster_indices.size();
  Ncluster = 1/Ncluster;

  // publish each cluster
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    // color each cluster with different color
    r = 255*((double)rand()/(double)RAND_MAX);
    g = 255*((double)rand()/(double)RAND_MAX);
    b = 255*((double)rand()/(double)RAND_MAX);
    rgb = (r << 16) | (g << 8) | b; 

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_cluster_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    int i = 0;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster_ptr->points.push_back (cloud_with_low_curvature_ptr->points[*pit]);
      cloud_cluster_ptr->points[i].rgb = *(float *)(&rgb);
      i++;
    }

    cloud_cluster_ptr->width = cloud_cluster_ptr->points.size();
    cloud_cluster_ptr->height = 1;
    cloud_cluster_ptr->is_dense = true;

    pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_cluster;
    cloud_cluster = *cloud_cluster_ptr;
    
    clusters.push_back(cloud_cluster);
    color_vec.at(0)=r;
    color_vec.at(1)=g;
    color_vec.at(2)=b;
    color_map[j]=color_vec;
    
    sensor_msgs::PointCloud2 cluster_cloud_msg;
    pcl::toROSMsg(cloud_cluster, cluster_cloud_msg);
    cluster_cloud_msg.header = cloud_msg.header;
    pub_cluster_cloud_.publish(cluster_cloud_msg);

    ROS_INFO( "Cluster %i: %lu points" , j, cloud_cluster.points.size() );

    j++;
  }

  

  ROS_INFO_STREAM("Processing time: " << ros::Time::now() - start_time << " seconds");

  //return;
  return true;
}


bool CurvatureFilter::footstep_place(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if(clusters.size()==0)
    {
	std::cout<<"No clusters to process, you should call the [/filter_by_curvature] service first"<<std::endl;
	return false;
    }
    
      
    for (unsigned int i=0; i< clusters.size(); i++)
    {
	std::cout<<std::endl;
	
	std::cout<<"Size of cluster "<<i<<": "<<clusters.at(i).size()<<std::endl;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);

	for (unsigned int j=0;j<clusters.at(i).size();j++)
	{
	      pcl::PointXYZ point;
	      point.x=clusters.at(i).at(j).x;
	      point.y=clusters.at(i).at(j).y;
	      point.z=clusters.at(i).at(j).z;
	      pCloud->push_back(point);
	}
	
	pcl::ConvexHull<pcl::PointXYZ> cHull;
	cHull.setDimension(3);
	cHull.setInputCloud(pCloud);
	
	pcl::PointCloud<pcl::PointXYZ> cHull_points;
	std::vector< pcl::Vertices > polygon;
	cHull.reconstruct (cHull_points,polygon);
	
	cHull.setComputeAreaVolume(true);
	
	std::cout<<"Evaluating convex hull . . . area : "<<cHull.getTotalArea()<<" , volume : "<<cHull.getTotalVolume()<<std::endl;
	std::cout<<"Convex hull pcl size: "<<cHull_points.size()<<std::endl;
	std::cout<<"Convex hull's number of facets: "<<polygon.size()<<std::endl;
	
	visualization_msgs::Marker marker;
	
	marker.header.frame_id="/camera_link";
	marker.ns="polygons";
	marker.id=i;
// 	marker.type=visualization_msgs::Marker::LINE_STRIP;
	marker.type=visualization_msgs::Marker::TRIANGLE_LIST;
	
	marker.scale.x=1;
	marker.scale.y=1;
	marker.scale.z=1;
	
	marker.color.a=1;
	
	geometry_msgs::Point point;
	
	std_msgs::ColorRGBA color;
	
	color.a=0.25;
// 	color.r=color_map.at(i).at(0);
// 	color.g=color_map.at(i).at(1);
// 	color.b=color_map.at(i).at(2);

	for (unsigned int j=0;j<polygon.size();j++) //ogni punto del poligono
	{
// 		marker.id=i*polygon.size()+j;
		
		pcl::Vertices vertices = polygon.at(j);
		
		color.r=255*((double)rand()/(double)RAND_MAX);
		color.g=255*((double)rand()/(double)RAND_MAX);
		color.b=255*((double)rand()/(double)RAND_MAX);
		
		//std::cout<<vertices<<std::endl;
		
		point.x = cHull_points.at(vertices.vertices.at(0)).x;
		point.y = cHull_points.at(vertices.vertices.at(0)).y;
		point.z = cHull_points.at(vertices.vertices.at(0)).z;
		
		marker.points.push_back(point);
		marker.colors.push_back(color);
		
		point.x = cHull_points.at(vertices.vertices.at(1)).x;
		point.y = cHull_points.at(vertices.vertices.at(1)).y;
		point.z = cHull_points.at(vertices.vertices.at(1)).z;
		
		marker.points.push_back(point);
		marker.colors.push_back(color);
		
		point.x = cHull_points.at(vertices.vertices.at(2)).x;
		point.y = cHull_points.at(vertices.vertices.at(2)).y;
		point.z = cHull_points.at(vertices.vertices.at(2)).z;
		
		marker.points.push_back(point);
		marker.colors.push_back(color);
		
// 		//to close the facet
// 		point.x = cHull_points.at(vertices.vertices.at(0)).x;
// 		point.y = cHull_points.at(vertices.vertices.at(0)).y;
// 		point.z = cHull_points.at(vertices.vertices.at(0)).z;
// 		
// 		marker.points.push_back(point);
		
		
	}
		
	pub_polygons_marker.publish(marker);
	
	std::cout<<"Convex hull's marker published"<<std::endl;
    }
    
    return true;
    
    std::cout<<"Placing feet in planes..."<<std::endl;
}


} // namespace plane_segmentation


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "object_modelling_node");
	ros::NodeHandle nh;

	plane_segmentation::CurvatureFilter node(nh);
  

  ros::Rate freq ( 1 );

  while(1)
  {
    ros::spinOnce();
    freq.sleep();
  }
	
	return 0;
}