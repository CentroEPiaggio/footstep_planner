#ifndef GRAHAM_SMITH_H
#define GRAHAM_SMITH_H
#include <pcl/point_types.h>
#include <kdl/frames.hpp>

class graham_smith
{
public:
    KDL::Frame createFramesFromNormal(pcl::PointXYZRGBNormal normal);
    void setCurrentDirection(KDL::Vector direction);
};

#endif // GRAHAM_SMITH_H
