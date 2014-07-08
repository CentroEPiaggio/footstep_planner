#ifndef BORDEREXTRACTION_H
#define BORDEREXTRACTION_H
#include <ros/publisher.h>
#include <ros/service_server.h>
#include <std_srvs/Empty.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "data_types.h"


using namespace planner;

class borderExtraction
{
public:
    
    std::list< polygon_with_normals > extractBorders(const std::vector< boost::shared_ptr< pcl::PointCloud< pcl::PointXYZRGBNormal > > >& clusters);
    
private:
    pcl::PointCloud< pcl::PointXYZ >::Ptr douglas_peucker_3d(pcl::PointCloud< pcl::PointXYZRGBNormal >& input, double tolerance);
    
};

#endif // BORDEREXTRACTION_H
