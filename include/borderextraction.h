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
    
    std::vector< std::shared_ptr< pcl::PointCloud< pcl::PointXYZ > > > extractBorders(const std::vector< boost::shared_ptr< pcl::PointCloud< pcl::PointXYZRGBNormal > > >& clusters);
    
private:
    bool douglas_peucker_3d(pcl::PointCloud< pcl::PointXYZ >& input, std::shared_ptr< pcl::PointCloud< pcl::PointXYZ > > output, double tolerance);
    
};


}
#endif // BORDEREXTRACTION_H
