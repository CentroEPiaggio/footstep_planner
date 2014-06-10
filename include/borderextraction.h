#ifndef BORDEREXTRACTION_H
#define BORDEREXTRACTION_H
#include <ros/publisher.h>
#include <ros/service_server.h>
#include <std_srvs/Empty.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace planner
{
    
    struct polygon_with_normals
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr border;
        pcl::PointCloud<pcl::PointNormal>::Ptr normals;
    };
    
    
    
class borderExtraction
{
public:
    
    std::vector< polygon_with_normals > extractBorders(const std::vector< boost::shared_ptr< pcl::PointCloud< pcl::PointXYZRGBNormal > > >& clusters);
    
private:
    pcl::PointCloud< pcl::PointXYZ >::Ptr douglas_peucker_3d(pcl::PointCloud< pcl::PointXYZ >& input, double tolerance);
    
};


}
#endif // BORDEREXTRACTION_H
