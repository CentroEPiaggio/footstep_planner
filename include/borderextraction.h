#ifndef BORDEREXTRACTION_H
#define BORDEREXTRACTION_H
#include <ros/publisher.h>
#include <ros/service_server.h>
#include <std_srvs/Empty.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace planner
{
    
class borderExtraction
{
public:
    
    std::vector< pcl::PointCloud<pcl::PointXYZ> > extractBorders(std::vector< pcl::PointCloud< pcl::PointXYZRGBNormal > >& clusters);
    
private:
    bool douglas_peucker_3d(pcl::PointCloud< pcl::PointXYZ >& input, pcl::PointCloud< pcl::PointXYZ >& output,double tolerance);
    
};


}
#endif // BORDEREXTRACTION_H
