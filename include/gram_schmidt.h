#ifndef gram_schmidt_H
#define gram_schmidt_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <kdl/frames_io.hpp>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/SVD>
#include <iostream>

class gram_schmidt
{

public:
gram_schmidt();
virtual ~gram_schmidt();

KDL::Frame createFramesFromNormal(pcl::PointXYZRGBNormal);
void setCurrentDirection(KDL::Vector direction);

private:
    Eigen::Vector3d proj_u(Eigen::Vector3d, Eigen::Vector3d);
    Eigen::Vector3d vd;
    bool direction_set=false;
    KDL::Frame frame_normal;
    
    template<typename _Matrix_Type_>
    static _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon =
    std::numeric_limits<double>::epsilon());
};

#endif // gram_schmidt_H
