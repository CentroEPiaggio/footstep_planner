#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace planner
{

struct polygon_with_normals
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr border;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normals;
    pcl::PointXYZRGBNormal average_normal;
};  
  
typedef struct
{
    int index;
    KDL::JntArray joints;
    KDL::Frame World_StanceFoot;
    KDL::Frame World_MovingFoot;
    KDL::Frame World_Waist;
}foot_with_joints;

}

#endif // DATA_TYPES_H
