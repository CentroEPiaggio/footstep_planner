#include <string>

// ROS headers
#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl/pcl_base.h>
#include <pcl/ModelCoefficients.h>

#include "MarkerGenerator.h"
#include "plane_segmentation/PlaneDetection.h"

namespace plane_segmentation {

class PlaneDetector
{
	typedef pcl::PointXYZRGB Point;
	
private:
	//! The node handle
	ros::NodeHandle nh_;
	//! Node handle in the private namespace
	ros::NodeHandle priv_nh_;

	//! Publisher for markers
	ros::Publisher marker_pub_;
	//! Service server for object detection
	ros::ServiceServer segmentation_srv_;

	//! Used to remember the number of markers we publish so we can delete them later
	int num_markers_published_;
	//! The current marker being published
	int current_marker_id_;

	//! Min number of inliers for reliable plane detection
	int inlier_threshold_;
	//! Size of downsampling grid before performing plane detection
	double plane_detection_voxel_size_;
	//! Size of downsampling grid before performing clustering
	double clustering_voxel_size_;

	//! Filtering of original point cloud along the z, y, and x axes
	double z_filter_min_, z_filter_max_;
	double y_filter_min_, y_filter_max_;
	double x_filter_min_, x_filter_max_;

	//! Positive or negative z is closer to the "up" direction in the processing frame?
	tf::Vector3 up_direction_;

	//! How much the table gets padded in the horizontal direction
	double plane_padding_;

	//! A tf transform listener
	//tf::TransformListener tf_listener_;

	//! A tf transform broadcaster
	tf::TransformBroadcaster tf_broadcaster_;

	//! The transform of the detected plane, in th future should be a vector of transforms for all detected planes
	tf::Transform plane_trans_;

	//------------------ Callbacks -------------------

	//! Callback for service calls
	bool detectPlanes(plane_segmentation::PlaneDetection::Request &request, plane_segmentation::PlaneDetection::Response &response);

	//------------------ Individual processing steps -------

	//! Converts raw plane detection results into a Plane message type
	template <class PointCloudType>
	plane_segmentation::Plane getPlaneMsg(std_msgs::Header cloud_header, const tf::Transform &plane_trans_, const PointCloudType &plane_points);

	//! Converts plane convex hull into a triangle mesh to add to a Plane message
	template <class PointCloudType>
	void publishConvexHullPlane(plane_segmentation::Plane &plane, const PointCloudType &convex_hull);

	//------------------- Complete processing -----

	//! Complete processing for new style point cloud
	void processCloud(const sensor_msgs::PointCloud2 &cloud, plane_segmentation::PlaneDetection::Response &response);
	
	//! Clears old published markers and remembers the current number of published markers
	void clearOldMarkers(std::string frame_id);

	//! Pull out and transform the convex hull points from a Plane message
	template <class PointCloudType>
	bool planeMsgToPointCloud (plane_segmentation::Plane &plane, std::string frame_id, PointCloudType &plane_hull);

public:

	//! Transform publisher
	void publishPlaneTransforms();

	PlaneDetector(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
	{
		num_markers_published_ = 1;
		current_marker_id_ = 1;

		// define the up direction, depends on the processing frame
		up_direction_ = tf::Vector3(0, 0, 1);

		marker_pub_ = nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("markers_out"), 10);

		segmentation_srv_ = nh_.advertiseService(nh_.resolveName("plane_detection_srv"), &PlaneDetector::detectPlanes, this);

		// initialize parameters if not loaded from launch file
		priv_nh_.param<int>("inlier_threshold", inlier_threshold_, 300);
		priv_nh_.param<double>("plane_detection_voxel_size", plane_detection_voxel_size_, 0.01);
		priv_nh_.param<double>("clustering_voxel_size", clustering_voxel_size_, 0.003);
		priv_nh_.param<double>("z_filter_min", z_filter_min_, 0.4);
		priv_nh_.param<double>("z_filter_max", z_filter_max_, 1.25);
		priv_nh_.param<double>("y_filter_min", y_filter_min_, -1.0);
		priv_nh_.param<double>("y_filter_max", y_filter_max_, 1.0);
		priv_nh_.param<double>("x_filter_min", x_filter_min_, -1.0);
		priv_nh_.param<double>("x_filter_max", x_filter_max_, 1.0);
		priv_nh_.param<double>("table_padding", plane_padding_, 0.0); 
	}

	//! Empty stub
	~PlaneDetector() {}
};

/*! Processes the latest point cloud and gives back the resulting array of models.
 */
bool PlaneDetector::detectPlanes(plane_segmentation::PlaneDetection::Request &request, plane_segmentation::PlaneDetection::Response &response)
{

	ros::Time start_time = ros::Time::now();

	// wait for a point cloud
	std::string topic = nh_.resolveName("cloud_in");
	ROS_INFO("plane_detector detection service called; waiting for a point_cloud2 on topic %s", topic.c_str());
	sensor_msgs::PointCloud2::ConstPtr recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh_, ros::Duration(3.0));
	if (!recent_cloud)
	{
		ROS_ERROR("plane_detector object detector: no point_cloud2 has been received");
		response.result = response.NO_CLOUD_RECEIVED;
		return true;
	}

	ROS_INFO_STREAM("Point cloud received after " << ros::Time::now() - start_time << " seconds; processing");
	
	// this function is the one to be looped, perhaps it has to output the cloud with the current plane
	processCloud(*recent_cloud, response);

	clearOldMarkers(recent_cloud->header.frame_id);

	//add the timestamp from the original cloud
	response.plane.pose.header.stamp = recent_cloud->header.stamp;
	
	ROS_INFO_STREAM("In total, detection took " << ros::Time::now() - start_time << " seconds");
	return true;
}

template <class PointCloudType>
void PlaneDetector::publishConvexHullPlane(plane_segmentation::Plane &plane, const PointCloudType &convex_hull)
{
	if (convex_hull.points.empty())
	{
		ROS_ERROR("Trying to add convex hull, but it contains no points");
		return;
	}

	//compute centroid
	geometry_msgs::Point centroid;
	centroid.x = centroid.y = centroid.z = 0.0;
	for (size_t i=0; i<convex_hull.points.size(); i++)
	{
		centroid.x += convex_hull.points[i].x;
		centroid.y += convex_hull.points[i].y;
		centroid.z += convex_hull.points[i].z;
	}
	centroid.x /= convex_hull.points.size();
	centroid.y /= convex_hull.points.size();
	centroid.z /= convex_hull.points.size();

	//create a triangle mesh out of the convex hull points and add it to the plane message
	for (size_t i=0; i<convex_hull.points.size(); i++)
	{
		geometry_msgs::Point vertex;
		vertex.x = convex_hull.points[i].x;
		vertex.y = convex_hull.points[i].y;
		if (plane_padding_ > 0.0)
		{
			double dx = vertex.x - centroid.x;
			double dy = vertex.y - centroid.y;
			double l = sqrt(dx*dx + dy*dy);
			dx /= l; dy /= l;
			vertex.x += plane_padding_ * dx;
			vertex.y += plane_padding_ * dy;
		}
		vertex.z = convex_hull.points[i].z;

		plane.convex_hull.vertices.push_back(vertex);
			
		if(i==0 || i==convex_hull.points.size()-1) continue;
		shape_msgs::MeshTriangle meshtri;
		meshtri.vertex_indices[0] = 0;
		meshtri.vertex_indices[1] = i;
		meshtri.vertex_indices[2] = i+1;
		
		plane.convex_hull.triangles.push_back(meshtri);

	}

	visualization_msgs::Marker planeMarker = MarkerGenerator::getConvexHullPlaneMarker(plane.convex_hull);
	planeMarker.header = plane.pose.header;
	planeMarker.pose = plane.pose.pose;
	planeMarker.ns = "plane_detector_node";
	planeMarker.id = current_marker_id_++;
	marker_pub_.publish(planeMarker);

	visualization_msgs::Marker originMarker = MarkerGenerator::createMarker(plane.pose.header.frame_id, 0, .0025, .0025, .01, 0, 1, 1, visualization_msgs::Marker::CUBE, current_marker_id_++, "plane_detector_node", plane.pose.pose);
	marker_pub_.publish(originMarker);
}

template <class PointCloudType>
plane_segmentation::Plane PlaneDetector::getPlaneMsg(std_msgs::Header cloud_header, const tf::Transform &plane_trans_,  const PointCloudType &plane_points)
{
	plane_segmentation::Plane plane;
 
	//get the extents of the plane
	if (!plane_points.points.empty()) 
	{
		plane.x_min = plane_points.points[0].x;
		plane.x_max = plane_points.points[0].x;
		plane.y_min = plane_points.points[0].y;
		plane.y_max = plane_points.points[0].y;
	}  
	for (size_t i=1; i<plane_points.points.size(); ++i) 
	{
		if (plane_points.points[i].x<plane.x_min && plane_points.points[i].x>-3.0) plane.x_min = plane_points.points[i].x;
		if (plane_points.points[i].x>plane.x_max && plane_points.points[i].x< 3.0) plane.x_max = plane_points.points[i].x;
		if (plane_points.points[i].y<plane.y_min && plane_points.points[i].y>-3.0) plane.y_min = plane_points.points[i].y;
		if (plane_points.points[i].y>plane.y_max && plane_points.points[i].y< 3.0) plane.y_max = plane_points.points[i].y;
	}

	geometry_msgs::Pose plane_pose;
	tf::poseTFToMsg(plane_trans_, plane_pose);
	plane.pose.pose = plane_pose;
	plane.pose.header = cloud_header;

	visualization_msgs::Marker planeMarker = MarkerGenerator::getPlaneMarker(plane.x_min, plane.x_max, plane.y_min, plane.y_max);
	planeMarker.header = cloud_header;
	planeMarker.pose = plane_pose;
	planeMarker.ns = "plane_detector_node";
	planeMarker.id = current_marker_id_++;
	marker_pub_.publish(planeMarker);
	
	return plane;
}

void PlaneDetector::clearOldMarkers(std::string frame_id)
{
	for (int id=current_marker_id_; id < num_markers_published_; id++)
		{
			visualization_msgs::Marker delete_marker;
			delete_marker.header.stamp = ros::Time::now();
			delete_marker.header.frame_id = frame_id;
			delete_marker.id = id;
			delete_marker.action = visualization_msgs::Marker::DELETE;
			delete_marker.ns = "plane_detector_node";
			marker_pub_.publish(delete_marker);
		}
	num_markers_published_ = current_marker_id_;
	current_marker_id_ = 0;
}

/*! Assumes plane coefficients are of the form ax+by+cz+d=0, normalized */
tf::Transform getPlaneTransform (pcl::ModelCoefficients coeffs, tf::Vector3 up_direction)
{
	ROS_ASSERT(coeffs.values.size() > 3);
	double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];

	//asume plane coefficients are normalized
	tf::Vector3 position(-a*d, -b*d, -c*d);
	tf::Vector3 z(a, b, c);

	//make sure z points "up"
	ROS_INFO("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
	if ( z.dot( up_direction ) < 0)
	{
		z = -1.0 * z;
		ROS_INFO("flipped z");
	}
		
	//try to align the x axis with the x axis of the original frame
	//or the y axis if z and x are too close too each other
	tf::Vector3 x(1, 0, 0);
	if ( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = tf::Vector3(0, 1, 0);
	tf::Vector3 y = z.cross(x).normalized();
	x = y.cross(z).normalized();

	tf::Matrix3x3 rotation;
	rotation[0] = x; 	// x
	rotation[1] = y; 	// y
	rotation[2] = z; 	// z
	rotation = rotation.transpose();
	tf::Quaternion orientation;
	rotation.getRotation(orientation);
	ROS_DEBUG("in getPlaneTransform, x: %0.3f, %0.3f, %0.3f", x[0], x[1], x[2]);
	ROS_DEBUG("in getPlaneTransform, y: %0.3f, %0.3f, %0.3f", y[0], y[1], y[2]);
	ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);

	return tf::Transform(orientation, position);
}

template <typename PointT> 
bool getPlanePoints (const pcl::PointCloud<PointT> &plane,  const tf::Transform& plane_trans_, sensor_msgs::PointCloud &plane_points)
{
	// Prepare the output
	plane_points.header = pcl_conversions::fromPCL(plane.header);
	plane_points.points.resize (plane.points.size ());
	for (size_t i = 0; i < plane.points.size (); ++i)
	{
		plane_points.points[i].x = plane.points[i].x;
		plane_points.points[i].y = plane.points[i].y;
		plane_points.points[i].z = plane.points[i].z;
	}

	// Transform the data
	tf::TransformListener listener;
	// ros::Time plane_stamp;
	// ros::Time now = ros::Time::now();
	// plane_stamp.fromNSec(plane.header.stamp*1000);
	// ROS_INFO("ROS TIME NOW %lu", now.toNSec()/1000);
	// ROS_INFO("plane HEADER STAMP %lu", plane_stamp.toNSec()/1000);
	// plane_stamp = pcl_conversions::fromPCL(plane.header.stamp);
	tf::StampedTransform plane_pose_frame(plane_trans_, plane_points.header.stamp, plane.header.frame_id, "plane_frame");
	listener.setTransform(plane_pose_frame);
	std::string error_msg;
	if (!listener.canTransform("plane_frame", plane_points.header.frame_id, plane_points.header.stamp, &error_msg))
	{
		ROS_ERROR("Can not transform point cloud from frame %s to plane frame; error %s", 
				plane_points.header.frame_id.c_str(), error_msg.c_str());
		return false;
	}
	int current_try = 0, max_tries = 3;
	while (1)
	{
		bool transform_success = true;
		try
		{
			listener.transformPointCloud("plane_frame", plane_points, plane_points);
		}
		catch (tf::TransformException ex)
		{
			transform_success = false;
			if ( ++current_try >= max_tries )
			{
				ROS_ERROR("Failed to transform point cloud from frame %s into plane_frame; error %s", 
									plane_points.header.frame_id.c_str(), ex.what());
				return false;
			}
			//sleep a bit to give the listener a chance to get a new transform
			ros::Duration(0.1).sleep();
		}
		if (transform_success) break;
	}
	plane_points.header.stamp = ros::Time::now();
	plane_points.header.frame_id = "plane_frame";
	return true;
}

void PlaneDetector::processCloud(const sensor_msgs::PointCloud2 &cloud, plane_segmentation::PlaneDetection::Response &response)
{
	ROS_INFO("Starting process on new cloud in frame %s", cloud.header.frame_id.c_str());

	// Step 0 : Convert the point cloud msg to a point cloud pcl
	pcl::PointCloud<Point>::Ptr cloud_ptr (new pcl::PointCloud<Point>); 
	pcl::fromROSMsg(cloud, *cloud_ptr);

	// Step 1 : Passthrough filter
	pcl::PassThrough<Point> psFilter;
	psFilter.setInputCloud (cloud_ptr);
	psFilter.setFilterFieldName ("z");
	psFilter.setFilterLimits (z_filter_min_, z_filter_max_);
	pcl::PointCloud<Point>::Ptr z_cloud_filtered_ptr (new pcl::PointCloud<Point>); 
	psFilter.filter (*z_cloud_filtered_ptr);

	psFilter.setInputCloud (z_cloud_filtered_ptr);
	psFilter.setFilterFieldName ("y");
	psFilter.setFilterLimits (y_filter_min_, y_filter_max_);
	pcl::PointCloud<Point>::Ptr y_cloud_filtered_ptr (new pcl::PointCloud<Point>); 
	psFilter.filter (*y_cloud_filtered_ptr);

	psFilter.setInputCloud (y_cloud_filtered_ptr);
	psFilter.setFilterFieldName ("x");
	psFilter.setFilterLimits (x_filter_min_, x_filter_max_);
	pcl::PointCloud<Point>::Ptr cloud_filtered_ptr (new pcl::PointCloud<Point>); 
	psFilter.filter (*cloud_filtered_ptr);
	
	// if (cloud_filtered_ptr->points.size() < (unsigned int)min_cluster_size_)
	// {
	//   ROS_INFO("Filtered cloud only has %d points", (int)cloud_filtered_ptr->points.size());
	//   response.result = response.NO_PLANES;
	//   return;
	// }

	ROS_INFO("Step 1 done");

	// Step 2 : Grid downsample
	pcl::PointCloud<Point>::Ptr cloud_downsampled_ptr (new pcl::PointCloud<Point>);
	pcl::VoxelGrid<Point> grid_;
	grid_.setLeafSize (plane_detection_voxel_size_, plane_detection_voxel_size_, plane_detection_voxel_size_);
	grid_.setFilterFieldName ("z");
	grid_.setFilterLimits (z_filter_min_, z_filter_max_);
	grid_.setDownsampleAllData (false);
	grid_.setInputCloud (cloud_filtered_ptr);
	grid_.filter (*cloud_downsampled_ptr);
	
	// if (cloud_downsampled_ptr->points.size() < (unsigned int)min_cluster_size_)
	// {
	//   ROS_INFO("Downsampled cloud only has %d points", (int)cloud_downsampled_ptr->points.size());
	//   response.result = response.NO_PLANES;    
	//   return;
	// }

	ROS_INFO("Step 2 done");

	// Step 3 : Estimate normals
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<Point>::Ptr normals_tree;
	normals_tree = boost::make_shared<pcl::search::KdTree<Point> > ();
	pcl::NormalEstimation<Point, pcl::Normal> normal_estimator;  
	normal_estimator.setKSearch (10);  
	normal_estimator.setSearchMethod (normals_tree);
	normal_estimator.setInputCloud (cloud_downsampled_ptr);
	normal_estimator.compute (*cloud_normals_ptr);

	ROS_INFO("Step 3 done");

	// Step 4 : Detect a plane
	pcl::PointIndices::Ptr plane_inliers_ptr (new pcl::PointIndices); 
	pcl::ModelCoefficients::Ptr plane_coefficients_ptr (new pcl::ModelCoefficients); 
	pcl::SACSegmentationFromNormals<Point, pcl::Normal> plane_model;
	plane_model.setDistanceThreshold (0.05); 
	plane_model.setMaxIterations (10000);
	plane_model.setNormalDistanceWeight (0.1);
	plane_model.setOptimizeCoefficients (true);
	plane_model.setModelType (pcl::SACMODEL_NORMAL_PLANE);
	plane_model.setMethodType (pcl::SAC_RANSAC);
	plane_model.setProbability (0.99);
	plane_model.setInputCloud (cloud_downsampled_ptr);
	plane_model.setInputNormals (cloud_normals_ptr);
	plane_model.segment (*plane_inliers_ptr, *plane_coefficients_ptr);

	if (plane_coefficients_ptr->values.size () <=3)
	{
		ROS_INFO("Failed to detect plane in scan");
		response.result = response.NO_PLANES;    
		return;
	}

	if ( plane_inliers_ptr->indices.size() < (unsigned int)inlier_threshold_)
	{
		ROS_INFO("Plane detection has %d inliers, below min threshold of %d", (int)plane_inliers_ptr->indices.size(), inlier_threshold_);
		response.result = response.NO_PLANES;
		return;
	}
	ROS_INFO ("Model found with %d inliers: [%f %f %f %f].", (int)plane_inliers_ptr->indices.size (), plane_coefficients_ptr->values[0], plane_coefficients_ptr->values[1], plane_coefficients_ptr->values[2], plane_coefficients_ptr->values[3]);

	ROS_INFO("Step 4 done");

	// Step 5 : Create the Plane msg
	pcl::PointCloud<Point>::Ptr plane_projected_ptr (new pcl::PointCloud<Point>);
	pcl::ProjectInliers<Point> plane_inliers;
	plane_inliers.setModelType (pcl::SACMODEL_PLANE);
	plane_inliers.setInputCloud (cloud_downsampled_ptr);
	plane_inliers.setIndices (plane_inliers_ptr);
	plane_inliers.setModelCoefficients (plane_coefficients_ptr);
	plane_inliers.filter (*plane_projected_ptr);
	
	// estimate the convex hull in camera frame
	pcl::PointCloud<Point>::Ptr plane_hull_ptr (new pcl::PointCloud<Point>); 
	sensor_msgs::PointCloud plane_hull_points;
	pcl::ConvexHull<Point> huller;
	huller.setInputCloud (plane_projected_ptr);
	huller.reconstruct (*plane_hull_ptr);

	// find the center of the convex hull and move the plane frame there
	plane_trans_ = getPlaneTransform (*plane_coefficients_ptr, up_direction_);
	tf::Vector3 plane_hull_centroid;
	double avg_x, avg_y, avg_z;
	avg_x = avg_y = avg_z = 0;
	for (size_t i=0; i<plane_projected_ptr->points.size(); i++)
	{
		avg_x += plane_projected_ptr->points[i].x;
		avg_y += plane_projected_ptr->points[i].y;
		avg_z += plane_projected_ptr->points[i].z;
	}
	avg_x /= plane_projected_ptr->points.size();
	avg_y /= plane_projected_ptr->points.size();
	avg_z /= plane_projected_ptr->points.size();
	ROS_INFO("average x,y,z = (%.5f, %.5f, %.5f)", avg_x, avg_y, avg_z);

	// place the new plane frame in the center of the convex hull
	plane_hull_centroid[0] = avg_x;
	plane_hull_centroid[1] = avg_y;
	plane_hull_centroid[2] = avg_z;
	plane_trans_.setOrigin(plane_hull_centroid);

	// fill the plane msg
	sensor_msgs::PointCloud plane_points;
	response.plane = getPlaneMsg<sensor_msgs::PointCloud>(cloud.header, plane_trans_, plane_points);

	// convert the convex hull points to flat plane frame
	if (!getPlanePoints<Point> (*plane_hull_ptr, plane_trans_, plane_hull_points))
	{
		response.result = response.OTHER_ERROR;
		return;
	}

	// add the convex hull as a triangle mesh to the plane message
	publishConvexHullPlane<sensor_msgs::PointCloud>(response.plane, plane_hull_points);
	
	ROS_INFO("Step 5 done");

	// // Step 6: Get the objects on top of the (non-flat) plane
	// pcl::PointIndices cloud_object_indices;
	// pcl::ExtractPolygonalPrismData<Point> prism_;
	// //prism_.setInputCloud (cloud_all_minus_plane_ptr);
	// prism_.setInputCloud (cloud_filtered_ptr);
	// prism_.setInputPlanarHull (plane_hull_ptr);
	// ROS_INFO("Using plane prism: %f to %f", plane_z_filter_min_, plane_z_filter_max_);
	// prism_.setHeightLimits (plane_z_filter_min_, plane_z_filter_max_);  
	// prism_.segment (cloud_object_indices);

	// pcl::PointCloud<Point>::Ptr cloud_objects_ptr (new pcl::PointCloud<Point>); 
	// pcl::ExtractIndices<Point> extract_object_indices;
	// extract_object_indices.setInputCloud (cloud_filtered_ptr);
	// extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
	// extract_object_indices.filter (*cloud_objects_ptr);

	// if (cloud_objects_ptr->points.empty()) 
	// {
	//   ROS_INFO("No objects on plane");
	//   //return;
	// }
	// else
	// {  
	//   ROS_INFO (" Number of object point candidates: %d.", (int)cloud_objects_ptr->points.size ());
	//   response.result = response.SUCCESS;
	//   // ---[ Downsample the points
	//   pcl::PointCloud<Point>::Ptr cloud_objects_downsampled_ptr (new pcl::PointCloud<Point>); 
	//   grid_objects_.setInputCloud (cloud_objects_ptr);
	//   grid_objects_.filter (*cloud_objects_downsampled_ptr);

	//   // ---[ If flattening the plane, adjust the points on the plane to be straight also
	//   if(true) straightenPoints<pcl::PointCloud<Point> >(*cloud_objects_downsampled_ptr, 
	//   							       plane_trans_, plane_trans_);

	//   ROS_INFO("Step 5 done");

	//   // Step 6: Split the objects into Euclidean clusters
	//   std::vector<pcl::PointIndices> clusters2;
	//   //pcl_cluster_.setInputCloud (cloud_objects_ptr);
	//   pcl_cluster_.setInputCloud (cloud_objects_downsampled_ptr);
	//   pcl_cluster_.extract (clusters2);
	//   ROS_INFO ("Number of clusters found matching the given constraints: %d.", (int)clusters2.size ());

	//   // ---[ Convert clusters into the PointCloud message
	//   std::vector<sensor_msgs::PointCloud> clusters;
	//   getClustersFromPointCloud2<Point> (*cloud_objects_downsampled_ptr, clusters2, clusters);
	//   ROS_INFO("Clusters converted");
	//   response.clusters = clusters;
		
	//   // FINAL, pusblish the markers
	//   publishClusterMarkers(clusters, cloud.header);
	// }    
}

void PlaneDetector::publishPlaneTransforms()
{
	// broadcast the plane transform if useful using tf_broadcaster_
	return;
}

} //namespace plane_segmentation

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "plane_detector_node");
	ros::NodeHandle nh;

	plane_segmentation::PlaneDetector node(nh);

	while(1)
	{	
		node.publishPlaneTransforms();
		ros::spin();  
	}
	
	return 0;
}
