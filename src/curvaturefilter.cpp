#include "curvaturefilter.h"
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <eigen3/Eigen/src/Core/Map.h>


using namespace planner;


void curvatureFilter::setParams(double curvature_threshold_,double voxel_size_, double normal_radius_, int min_cluster_size_, double cluster_tolerance_)
{
    this->cluster_tolerance_=cluster_tolerance_;
    this->curvature_threshold_=curvature_threshold_;
    this->voxel_size_=voxel_size_;
    this->normal_radius_=normal_radius_;
    this->min_cluster_size_=min_cluster_size_;
}

// custom condition for euclidean clustering
// for instance check the difference of the normals in clusters
bool enforceCurvature (const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance)
{
    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
    
    // TODO: avoid this magic number here!
    if (fabs (point_a_normal.dot (point_b_normal)) > 0.9995)
    {
        return (true);
    }
    return (false);
}

//void CurvatureFilter::filterByCurvature(const sensor_msgs::PointCloud2 & input)
std::vector< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr >  curvatureFilter::filterByCurvature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr)
{
    ros::Time start_time = ros::Time::now();
    
    std::vector< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr >  clusters;
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
    
    // // write file to check normals
    // pcl::io::savePCDFileBinaryCompressed("scene_with_normals.pcd", cloud_with_normals);
    // // write file to check normals
    
    
    int N = (int) cloud_with_normals.points.size();
    printf("Cloud size is %i", N);
    
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
            
            //std::cout << " cloud_with_normals.points[i].curvature: " <<  cloud_with_normals.points[i].curvature << std::endl;
        }
    }
    
/*     // publish the colored cloud
//     sensor_msgs::PointCloud2 cloud_msg;
//     pcl::toROSMsg(cloud_with_normals, cloud_msg);
//     pub_colored_cloud_.publish(cloud_msg);
//     
//     cloud_with_low_curvature = *cloud_with_low_curvature_ptr;
//     // publish the cloud with planar regions only
//     sensor_msgs::PointCloud2 another_cloud_msg;
//     pcl::toROSMsg(cloud_with_low_curvature, another_cloud_msg);
//     another_cloud_msg.header = cloud_msg.header;
//     pub_filtered_cloud_.publish(another_cloud_msg);*/
    
    
    // perform conditional euclidean clustering on the cloud with planar areas only
    // we need another tree because this cloud is different
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr another_tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    std::vector<pcl::PointIndices> cluster_indices;
    //pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormal> ec;
    pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> ec;
    ec.setClusterTolerance (cluster_tolerance_);
    ec.setMinClusterSize (min_cluster_size_);
    ec.setMaxClusterSize (10000000); // a point cloud only has around 300000 so we are safe here
    //ec.setSearchMethod (another_tree);
    ec.setInputCloud (cloud_with_low_curvature_ptr);
    ec.setConditionFunction(enforceCurvature);
    // ec.extract (cluster_indices);
    ec.segment (cluster_indices);
    
    printf( "Found %lu clusters", cluster_indices.size() );
    
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
        
        //pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_cluster;
        //cloud_cluster = *cloud_cluster_ptr;
        
        clusters.push_back(cloud_cluster_ptr);
        
        j++;
//         if (j>10 ) break; //HACK 
    }
    
    
    
    ROS_INFO_STREAM("Processing time: " << ros::Time::now() - start_time << " seconds");
    
    //return;
    return clusters;
}