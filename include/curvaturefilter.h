#ifndef CURVATUREFILTER_H
#define CURVATUREFILTER_H
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "ros_publisher.h"

namespace planner
{

class curvatureFilter
{
public:
    std::vector< pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr >  filterByCurvature(ros_publisher* publish,pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr);
    void setParams(double curvature_threshold_,double voxel_size_, double normal_radius_, int min_cluster_size_, double cluster_tolerance_);
private:
    // normal estimator radius
    double curvature_threshold_;
    
    // downsample
    double voxel_size_;
    
    // normal estimator radius
    double normal_radius_;
    
    // euclidean cluster min size (related to the area, since we are filtering only planar regions on a grid)
    int min_cluster_size_;
    
    // euclidean cluster tolerance
    double cluster_tolerance_;
};


}
#endif // CURVATUREFILTER_H
