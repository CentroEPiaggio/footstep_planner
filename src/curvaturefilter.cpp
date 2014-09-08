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

#include "curvaturefilter.h"
#include <ros_publisher.h>
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

std::vector< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr >  curvatureFilter::filterByCurvature(ros_publisher* publish,pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr)
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
    ne.setViewPoint(0,0,0);
    ne.setRadiusSearch (normal_radius_);
    ne.compute (*cloud_normals_ptr);
    
//     int Normals = (int) cloud_normals_ptr->points.size();
// 
//     pcl::PointCloud<pcl::Normal>::Ptr show_normals(new pcl::PointCloud<pcl::Normal>());
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_points(new pcl::PointCloud<pcl::PointXYZRGB>());
//     
//     for (int i=0;i<Normals;i++)
//     {
//       if (cloud_downsampled_ptr->at(i).x>0.5 && cloud_downsampled_ptr->at(i).z<1.4)//
//       {
// 	show_normals->push_back(cloud_normals_ptr->at(i));
// 	show_points->push_back(cloud_downsampled_ptr->at(i));
//       }
//     }
    //publish->publish_normal_cloud(show_normals,show_points,0);
    //publish->publish_normal_cloud(cloud_normals_ptr,cloud_downsampled_ptr,0);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
    pcl::concatenateFields( *cloud_downsampled_ptr, *cloud_normals_ptr, *cloud_with_normals_ptr );
    pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_with_normals;
    cloud_with_normals = *cloud_with_normals_ptr;
    
    
    int N = (int) cloud_with_normals.points.size();
    printf("Cloud size is %i", N);
    // perform conditional euclidean clustering on the cloud with planar areas only
    // we need another tree because this cloud is different
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr another_tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> ec;
    ec.setClusterTolerance (cluster_tolerance_);
    ec.setMinClusterSize (min_cluster_size_);
    ec.setMaxClusterSize (10000000); // a point cloud only has around 300000 so we are safe here
    ec.setInputCloud (cloud_with_normals_ptr);
    ec.setConditionFunction(enforceCurvature);
    ec.segment (cluster_indices);
    
    printf( "Found %lu clusters", cluster_indices.size() );
    
    int Ncluster = (int)cluster_indices.size();
    Ncluster = 1/Ncluster;
    
    // for coloring the clouds
    uint8_t r, g, b;
    int32_t rgb; 
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
            cloud_cluster_ptr->points.push_back (cloud_with_normals_ptr->points[*pit]);
            cloud_cluster_ptr->points[i].rgb = *(float *)(&rgb);
            i++;
        }
        
        cloud_cluster_ptr->width = cloud_cluster_ptr->points.size();
        cloud_cluster_ptr->height = 1;
        cloud_cluster_ptr->is_dense = true;        
        clusters.push_back(cloud_cluster_ptr);
        
        j++;
    }
    
//     int i=0;
//     for (auto polygon:clusters)
//         publish->publish_normal_cloud(polygon,i++);
    
    
    ROS_INFO_STREAM("Processing time: " << ros::Time::now() - start_time << " seconds");
    
//     sleep(10);
    return clusters;
}