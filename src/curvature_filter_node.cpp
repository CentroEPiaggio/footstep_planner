// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>


// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/boundary.h>

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
      pub_footstep = nh_.advertise<visualization_msgs::Marker>("/footstep_marker",1,this);

     	//sub_input_cloud_ = nh_.subscribe(nh_.resolveName("input_cloud"), 100, &CurvatureFilter::filterByCurvature, this);

      srv_filter_cloud_ = nh_.advertiseService(nh_.resolveName("filter_by_curvature"), &CurvatureFilter::filterByCurvature, this);
      srv_convex_hull = nh_.advertiseService(nh_.resolveName("convex_hull"), &CurvatureFilter::convex_hull, this);
      srv_border_extraction = nh_.advertiseService(nh_.resolveName("border_extraction"), &CurvatureFilter::border_extraction, this);
      srv_footstep_placer = nh_.advertiseService(nh_.resolveName("footstep_placer"),&CurvatureFilter::footstep_placer, this);
      
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


bool CurvatureFilter::convex_hull(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
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
	std::cout<<"Convex hull pcl number of points: "<<cHull_points.size()<<std::endl;
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
	
	Eigen::Matrix<double,4,1> centroid;
	
	if(pcl::compute3DCentroid(cHull_points, centroid ))
	{
		std::cout<<"Computing convex hull's centroid . . . "<<std::endl;
		marker.ns="centroids";
		marker.id=i;
		marker.type=visualization_msgs::Marker::SPHERE;
		
		marker.scale.x=0.05;
		marker.scale.y=0.05;
		marker.scale.z=0.05;
		
		marker.color.a=1;
		marker.color.r=1;
		marker.color.g=0;
		marker.color.b=0;
		
		marker.pose.position.x=centroid[0];
		marker.pose.position.y=centroid[1];
		marker.pose.position.z=centroid[2];
		
		marker.pose.orientation.w=1;
		marker.pose.orientation.x=0;
		marker.pose.orientation.y=0;
		marker.pose.orientation.z=0;
		
		pub_polygons_marker.publish(marker);
	}
	else std::cout<<"Error computing convex hull's centroid!"<<std::endl;
    }
    
    return true;
}


bool compare_2d(pcl::PointXYZ a, pcl::PointXYZ b)
{
    pcl::PointXYZ center;
    center.z=0.0;
    center.x = (a.x + b.x)/2.0; 
    center.y = (a.y + b.y)/2.0; 
    
    if (a.x - center.x >= 0 && b.x - center.x < 0)
        return true;
    if (a.x - center.x < 0 && b.x - center.x >= 0)
        return false;
    if (a.x - center.x == 0 && b.x - center.x == 0) {
        if (a.y - center.y >= 0 || b.y - center.y >= 0)
            return a.y > b.y;
        return b.y > a.y;
    }

    // compute the cross product of vectors (center -> a) x (center -> b)
    int det = (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
    if (det < 0)
        return true;
    if (det > 0)
        return false;

    // points a and b are on the same line from the center
    // check which point is closer to the center
    int d1 = (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y);
    int d2 = (b.x - center.x) * (b.x - center.x) + (b.y - center.y) * (b.y - center.y);
    return d1 > d2;
}

bool neg_compare_2d(pcl::PointXYZ a, pcl::PointXYZ b)
{
    return !(compare_2d(a,b));
}

bool atan_compare_2d(pcl::PointXYZ a, pcl::PointXYZ b)
{
    return atan2(a.y,a.x) < atan2(b.y,b.x);
}

bool CurvatureFilter::douglas_peucker_3d(pcl::PointCloud< pcl::PointXYZ >& input, pcl::PointCloud< pcl::PointXYZ >& output,double tolerance)
{  
    if(!input.size()) return false;
    
    std::sort(input.begin(),input.end(),atan_compare_2d); //sorting input for douglas_peucker_3d procedure

    
    std::vector <double> pcl_vector;
    
    for(unsigned int i=0;i<input.size();i++) {pcl_vector.push_back(input.at(i).x);pcl_vector.push_back(input.at(i).y);pcl_vector.push_back(input.at(i).z);};
    
    double* result = new double[pcl_vector.size()];
    
    for(unsigned int h=0;h<pcl_vector.size();h++) result[h]=0.0;    
    
    double* iter = psimpl::simplify_douglas_peucker <3> (pcl_vector.begin(), pcl_vector.end(), tolerance, result);
    //double* iter = psimpl::simplify_douglas_peucker_n<3>(pcl_vector.begin(), pcl_vector.end(), 50, result); //variant
    
    pcl::PointXYZ point;
    
    unsigned int j=0;
    bool control=false;
    unsigned int i;
    
    for(i=0;i<pcl_vector.size();i++)
    {
          if(&result[i] == iter) break;
	  
	  if(!control && j==0)
	  {
		point.x = result[i];
		j++;
		control=true;
	  }
	  
	  if(!control && j==1)
	  {
		point.y = result[i];
		j++;
		control=true;
	  }
	  
	  if(!control && j==2)
	  {
		point.z = result[i];
		j=0;
		output.push_back(point);
	  }
	  
	  control=false;
    }
    
    std::cout<<"- INFO: i = "<<i<<std::endl;
    if(j!=0) {std::cout<<"- !! Error in output dimension (no 3d) !!"<<std::endl;}
    
    return true;
}

bool CurvatureFilter::border_extraction(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if(clusters.size()==0)
    {
	std::cout<<"No clusters to process, you should call the [/filter_by_curvature] service first"<<std::endl;
	return false;
    }
    
    geometry_msgs::Point point;
    
    visualization_msgs::Marker marker;
    
    marker.header.frame_id="/camera_link";
    marker.ns="borders";
    marker.type=visualization_msgs::Marker::SPHERE_LIST;
    
    marker.pose.position.x=0;
    marker.pose.position.y=0;
    marker.pose.position.z=0;
    marker.pose.orientation.w=1;
    marker.pose.orientation.x=0;
    marker.pose.orientation.y=0;
    marker.pose.orientation.z=0;
    
    marker.scale.x=0.01;
    marker.scale.y=0.01;
    marker.scale.z=0.01;
    marker.color.a=1;
    
    visualization_msgs::Marker marker2;

    marker2.header.frame_id="/camera_link";
    marker2.ns="border_poly";
    marker2.type=visualization_msgs::Marker::SPHERE_LIST;
    //marker2.type=visualization_msgs::Marker::LINE_STRIP;
    
    marker2.pose.position.x=0;
    marker2.pose.position.y=0;
    marker2.pose.position.z=0;
    marker2.pose.orientation.w=1;
    marker2.pose.orientation.x=0;
    marker2.pose.orientation.y=0;
    marker2.pose.orientation.z=0;
    
    marker2.scale.x=0.02;
    marker2.scale.y=0.02;
    marker2.scale.z=0.02;
    marker2.color.a=1;
      
    for (unsigned int i=0; i< clusters.size(); i++)
    {
	std::cout<<std::endl;
	
	std::cout<<"- Size of cluster "<<i<<": "<<clusters.at(i).size()<<std::endl;
	
        pcl::PointCloud<pcl::Boundary> boundaries; 
        pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; 
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; 
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); 

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); 
	
	pcl::PointXYZ pcl_point;
	std_msgs::ColorRGBA color;
	color.r=clusters.at(i).at(0).r;
	color.g=clusters.at(i).at(0).g;
	color.b=clusters.at(i).at(0).b;
		
	
	color.a=0.25;
	
	for(unsigned int pc=0; pc<clusters.at(i).size();pc++)
	{
	        pcl_point.x = clusters.at(i).at(pc).x;
	        pcl_point.y = clusters.at(i).at(pc).y;
		pcl_point.z = clusters.at(i).at(pc).z;
				
		cloud->points.push_back(pcl_point);
	}
	
        normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud)); 
        normEst.setRadiusSearch(0.1); 
        normEst.compute(*normals); 

        boundEst.setInputCloud(cloud); 
        boundEst.setInputNormals(normals); 
        boundEst.setRadiusSearch(0.1); 
        boundEst.setAngleThreshold(M_PI/4); 
        boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); 
	
	std::cout<<"- Estimating border from cluster . . ."<<std::endl;
	
        boundEst.compute(boundaries); 
	
	pcl::PointCloud<pcl::PointXYZ> border;

        for(int b = 0; b < cloud->points.size(); b++) 
        { 
                if(boundaries[b].boundary_point < 1) 
                { 
                        //not in the boundary
                } 
                else
	        {  
			point.x = cloud->at(b).x;
			point.y = cloud->at(b).y;
			point.z = cloud->at(b).z;
			
			border.push_back(cloud->at(b));
			
			marker.colors.push_back(color);
			marker.points.push_back(point);
		}
        } 
        
	marker.id=i;
	
	std::cout<<"- Border number of points: "<<border.size()<<std::endl;
	
	pub_border_marker.publish(marker);
	
	pcl::PointCloud<pcl::PointXYZ> border_polygon;
	
	std::cout<<"- Computing polygon which approximate the border . . ."<<std::endl;
	
	if(!douglas_peucker_3d(border,border_polygon,0.05)){ std::cout<<"- !! Failed to Compute the polygon to approximate the Border !!"<<std::endl; return false;}
	
	std::cout<<"- Polygon number of points: "<<border_polygon.size()<<std::endl;
	
	polygons.push_back(border_polygon);
	
	for(int po = 0; po < border_polygon.points.size(); po++) 
        { 
                point.x = border_polygon.at(po).x;
		point.y = border_polygon.at(po).y;
		point.z = border_polygon.at(po).z;
		
		marker2.colors.push_back(color);
		marker2.points.push_back(point);
        }
        
        marker2.id=i;
        
        pub_border_poly_marker.publish(marker2);
    }
    
    return true;
}


bool CurvatureFilter::footstep_placer(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
        if(polygons.size()==0) {std::cout<<"No polygons to process, you should call the [/filter_by_curvature] and [/border_extraction] services first"<<std::endl; return false;}
	std::cout<<"> Number of polygons: "<<polygons.size()<<std::endl;
	
	visualization_msgs::Marker marker;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.header.frame_id="/camera_link";
	marker.scale.x=0.1;
	marker.scale.y=0.05;
	marker.scale.z=0.02;
	marker.ns = "feet";
	marker.color.a=1;
	marker.color.b=0;
	bool left=true;
	
	Eigen::Matrix<double,4,1> centroid;
	
	for(unsigned int i=0;i<polygons.size();i++)
	{
		 std::cout<<"> Polygon number of points: "<<polygons.at(i).size()<<std::endl;
		
		
		//for now we place the foot in the centroid of the polygon (safe if convex)
		
		if(pcl::compute3DCentroid(polygons.at(i), centroid ))
		{
			if(left){ marker.color.r=1; marker.color.g=0; left=false;}
			else { marker.color.r=0; marker.color.g=1; left=true;}
			
			marker.pose.position.x=centroid[0];
			marker.pose.position.y=centroid[1];
			marker.pose.position.z=centroid[2];
			
			marker.pose.orientation.w=1; //TODO: orientation (x,y) as the plane where is placed!
			marker.pose.orientation.x=0;
			marker.pose.orientation.y=0;
			marker.pose.orientation.z=0;
			
			marker.id = i;
			
			std::cout<<"> Foot Placed in the centroid of the polygon"<<std::endl;
			
			pub_footstep.publish(marker);
			
		} else std::cout<<"!! Error in computing the centroid !!"<<std::endl;
	}
	
	return true;
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